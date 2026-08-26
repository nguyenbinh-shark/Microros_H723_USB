#!/usr/bin/env python3
"""
teleop_key.py — Điều khiển robot bằng bàn phím cho STM32 micro-ROS node (ROS 2 Jazzy).

Publish geometry_msgs/Twist lên topic /cmd_vel. STM32 subscribe /cmd_vel
(xem Core/Src/freertos.c::cmd_vel_callback) rồi đổi thành lệnh cho 2 động cơ.

═══════════════════════ LƯU Ý AN TOÀN ═══════════════════════
Trong firmware hiện tại (motor_task.c, lỗi #4 chưa sửa), giá trị linear.x và
angular.z được dùng TRỰC TIẾP làm MOMENT LỰC (Nm) — KHÔNG phải vận tốc (m/s).
=> Đây thực chất là điều khiển moment hở (open-loop torque). Vì vậy:
   • Bắt đầu với step nhỏ (mặc định 0.05).
   • Có phím space để DỪNG ngay.
   • Có "deadman": tự gửi 0 (dừng) nếu bạn không nhấn phím trong --idle giây.
     Tránh runaway khi terminal mất focus hoặc bạn rời tay.
══════════════════════════════════════════════════════════════

Cách chạy (trên PC, ROS 2 Jazzy):
    source /opt/ros/jazzy/setup.bash
    python3 teleop_key.py                       # tham số mặc định (an toàn)
    python3 teleop_key.py --step 0.1 --max 1.0  # tăng lực nếu cần
    python3 teleop_key.py --idle 0              # TẮT deadman (cẩn thận!)

Yêu cầu: micro-ROS agent đang chạy trên PC để /cmd_vel được bridge tới STM32.
"""

import argparse
import queue
import select
import sys
import termios
import threading
import time
import tty

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist


# ─── Tham số mặc định (Điều khiển Vận tốc chuẩn m/s và rad/s) ───────────────
DEFAULT_LINEAR_STEP = 0.05    # m/s mỗi lần nhấn tiến/lùi (5 cm/s)
DEFAULT_ANGULAR_STEP = 0.10   # rad/s mỗi lần nhấn quay
DEFAULT_MAX_LINEAR = 0.80     # Vận tốc tiến lùi tối đa (m/s)
DEFAULT_MAX_ANGULAR = 1.50    # Vận tốc quay tối đa (rad/s)
DEFAULT_RATE_HZ = 20.0        # Hz publish /cmd_vel
DEFAULT_IDLE_STOP_S = 0.0     # 0 = Giữ vận tốc cho đến khi nhấn Space để dừng

DEFAULT_TOPIC = '/cmd_vel'

HELP_TEXT = """\
---------------- Dieu khien robot (/cmd_vel: m/s & rad/s) ----------------
  w / W      : tien thang       (linear  += 0.05 m/s)
  s / S      : lui              (linear  -= 0.05 m/s)
  a / A      : quay trai        (angular += 0.10 rad/s)
  d / D      : quay phai        (angular -= 0.10 rad/s)
  space      : DUNG NGAY        (0.0 m/s)
  + / =      : tang buoc step   (x1.25)
  - / _      : giam buoc step   (/1.25)
  q / Ctrl-C : thoat
--------------------------------------------------------------------------
 (*) Don vi: linear.x = [m/s], angular.z = [rad/s].
 (*) STM32 tu dong chay Kinematics & Closed-loop Velocity FOC tren chip.
"""


def clamp(v, lo, hi):
    """Kẹp giá trị v vào khoảng [lo, hi]."""
    return max(lo, min(hi, v))


class KeyReader(threading.Thread):
    """Đọc phím ở chế độ raw (non-blocking) trong một luồng riêng.

    Terminal được đặt về raw mode để nhận từng ký tự ngay (không cần Enter,
    không echo). Khi dừng, khôi phục lại cấu hình terminal cũ.
    """

    def __init__(self, key_q: queue.Queue, stop_event: threading.Event):
        super().__init__(daemon=True)
        self.key_q = key_q
        self.stop_event = stop_event

    def run(self):
        fd = sys.stdin.fileno()
        old_attrs = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while not self.stop_event.is_set():
                ready, _, _ = select.select([sys.stdin], [], [], 0.1)
                if ready:
                    ch = sys.stdin.read(1)
                    if ch:
                        self.key_q.put(ch)
        finally:
            # Luôn khôi phục terminal để không "phá" console
            termios.tcsetattr(fd, termios.TCSADRAIN, old_attrs)


def main():
    parser = argparse.ArgumentParser(
        description='Ban phim -> /cmd_vel (ROS 2 Jazzy, micro-ROS)')
    parser.add_argument('--step', type=float, default=DEFAULT_LINEAR_STEP,
                        help='Buoc tien/lui moi phim (Nm). Mac dinh: %(default)s')
    parser.add_argument('--astep', type=float, default=DEFAULT_ANGULAR_STEP,
                        help='Buoc goc moi phim (Nm). Mac dinh: %(default)s')
    parser.add_argument('--max', type=float, default=DEFAULT_MAX_LINEAR,
                        help='Gioi han |linear| (Nm). Mac dinh: %(default)s')
    parser.add_argument('--amax', type=float, default=DEFAULT_MAX_ANGULAR,
                        help='Gioi han |angular| (Nm). Mac dinh: %(default)s')
    parser.add_argument('--rate', type=float, default=DEFAULT_RATE_HZ,
                        help='Tan so publish (Hz). Mac dinh: %(default)s')
    parser.add_argument('--idle', type=float, default=DEFAULT_IDLE_STOP_S,
                        help='Auto-dung sau N giay khong nhan (0=tat deadman). '
                             'Mac dinh: %(default)s')
    parser.add_argument('--topic', type=str, default=DEFAULT_TOPIC,
                        help='Topic xuat. Mac dinh: %(default)s')
    args = parser.parse_args()

    if not sys.stdin.isatty():
        print('Loi: can chay trong terminal (tty) de doc phim.', file=sys.stderr)
        sys.exit(1)

    rclpy.init()
    node = Node('teleop_key')
    pub = node.create_publisher(Twist, args.topic, 10)

    print(HELP_TEXT.format(idle=args.idle))
    print("[1/2] Đang chờ kết nối với STM32 (DDS Discovery)...")
    sys.stdout.flush()

    # Chờ cho đến khi Publisher của PC tìm thấy Subscriber /cmd_vel trên STM32
    while rclpy.ok() and pub.get_subscription_count() == 0:
        rclpy.spin_once(node, timeout_sec=0.1)
        time.sleep(0.1)

    print("\n[2/2] ==> ĐÃ KẾT NỐI VỚI STM32 THÀNH CÔNG! Bấm W/S/A/D để chạy.\n")
    sys.stdout.flush()

    key_q: queue.Queue = queue.Queue()
    stop_event = threading.Event()
    reader = KeyReader(key_q, stop_event)
    reader.start()

    linear = 0.0
    angular = 0.0
    step = args.step
    astep = args.astep
    last_key_time = time.monotonic()
    running = True

    twist = Twist()
    period = 1.0 / args.rate if args.rate > 0 else 0.05

    def publish_zero():
        """Gửi Twist 0 để dừng động cơ (STM32 sẽ disable motor khi =0)."""
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)

    def print_state():
        sys.stdout.write(
            '\r>> v={:+.2f} m/s  w={:+.2f} rad/s  [step={:.2f} m/s]   '.format(
                linear, angular, step))
        sys.stdout.flush()

    try:
        while running and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0)
            now = time.monotonic()
            changed = False

            # Xử lý tất cả phím đã được luồng đọc đưa vào hàng đợi
            while not key_q.empty():
                try:
                    k = key_q.get_nowait()
                except queue.Empty:
                    break
                last_key_time = now
                if k in ('q', 'Q', '\x03'):              # q hoặc Ctrl-C
                    running = False
                    break
                elif k == ' ':
                    linear = 0.0
                    angular = 0.0
                    changed = True
                elif k in ('w', 'W'):
                    linear += step
                    changed = True
                elif k in ('s', 'S'):
                    linear -= step
                    changed = True
                elif k in ('a', 'A'):
                    angular += astep
                    changed = True
                elif k in ('d', 'D'):
                    angular -= astep
                    changed = True
                elif k in ('+', '='):
                    step = round(step * 1.25, 3)
                    astep = round(astep * 1.25, 3)
                    changed = True
                elif k in ('-', '_'):
                    step = round(max(0.05, step / 1.25), 3)
                    astep = round(max(0.05, astep / 1.25), 3)
                    changed = True

            # Deadman (chỉ kích hoạt nếu --idle > 0)
            if args.idle > 0 and (now - last_key_time) > args.idle:
                if linear != 0.0 or angular != 0.0:
                    linear = 0.0
                    angular = 0.0
                    changed = True

            # Giới hạn an toàn
            linear = round(clamp(linear, -args.max, args.max), 3)
            angular = round(clamp(angular, -args.amax, args.amax), 3)

            # Publish khi có thay đổi hoặc khi đang chạy
            if changed or (linear != 0.0 or angular != 0.0):
                twist.linear.x = float(linear)
                twist.angular.z = float(angular)
                pub.publish(twist)

            if changed:
                print_state()

            time.sleep(period)

    except KeyboardInterrupt:
        pass
    finally:
        # QUAN TRỌNG: gửi lệnh dừng trước khi thoát để động cơ không chạy tiếp
        try:
            publish_zero()
        except Exception:
            pass
        stop_event.set()
        reader.join(timeout=1.0)
        node.destroy_node()
        rclpy.try_shutdown()
        print('\nDa thoat. Da gui lenh dung (zero) len /cmd_vel.')


if __name__ == '__main__':
    main()
