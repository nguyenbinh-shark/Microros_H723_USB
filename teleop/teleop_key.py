#!/usr/bin/env python3
"""
teleop_key.py — Dieu khien robot bang ban phim cho STM32 micro-ROS node (ROS 2 Jazzy).

Publish geometry_msgs/Twist len topic /cmd_vel va std_msgs/Bool len topic /motor_enable.
STM32 subscribe /cmd_vel roi doi thanh lenh van toc cho 2 dong co qua he vi sai (Kinematics).

======================= LUU Y AN TOAN =======================
1. Dong co mac dinh o trang thai DISABLE khi khoi dong. Nhan 'e' de ENABLE.
2. linear.x (m/s) va angular.z (rad/s) duoc dieu khien vong kin (Closed-loop Velocity).
3. Co phim space de DUNG NGAY (v=0, w=0).
4. Co 'deadman' (mac dinh --idle 0.5 giay): tu dong gui lenh dung neu khong nhan phim.
=============================================================

Cach chay (tren PC / Raspberry Pi voi ROS 2 Jazzy):
    source /opt/ros/jazzy/setup.bash
    python3 teleop/teleop_key.py                       # mac dinh (an toan)
    python3 teleop/teleop_key.py --step 0.2 --max 1.0  # tuy chinh buoc toc
    python3 teleop/teleop_key.py --idle 0              # TAT deadman (can than!)
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
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


# --- Tham so mac dinh (m/s va rad/s) ----------------------------------------
DEFAULT_LINEAR_STEP = 0.10    # m/s moi lan nhan tien/lui
DEFAULT_ANGULAR_STEP = 0.10   # rad/s moi lan nhan quay
DEFAULT_MAX_LINEAR = 0.80     # Van toc tien/lui toi da (m/s)
DEFAULT_MAX_ANGULAR = 1.50    # Van toc quay toi da (rad/s)
DEFAULT_RATE_HZ = 20.0        # Tan so publish /cmd_vel (Hz)
DEFAULT_IDLE_STOP_S = 0.5     # Deadman: tu dung sau 0.5s neu khong nhan phim (--idle 0 de tat)

DEFAULT_CMD_TOPIC = '/cmd_vel'
DEFAULT_ENABLE_TOPIC = '/motor_enable'

HELP_TEXT = """\
---------------- Dieu khien robot (/cmd_vel: m/s & rad/s) ----------------
  w / W      : tien thang       (linear  += step)
  s / S      : lui              (linear  -= step)
  a / A      : quay trai        (angular += astep)
  d / D      : quay phai        (angular -= astep)
  space      : DUNG NGAY        (linear=0, angular=0)
  e / E      : ENABLE motor     (/motor_enable = True)
  x / X      : DISABLE motor    (/motor_enable = False)
  + / =      : tang buoc step   (x1.25)
  - / _      : giam buoc step   (/1.25)
  q / Ctrl-C : thoat
--------------------------------------------------------------------------
 (*) Chu y: Nhan 'e' de ENABLE dong co truoc khi chay!
 (*) Deadman: Tu dong dung sau {idle:.1f}s neu khong giu phim (0 = tat).
 (*) Don vi: linear.x = [m/s], angular.z = [rad/s].
"""


def clamp(v: float, lo: float, hi: float) -> float:
    """Kep gia tri v vao khoang [lo, hi]."""
    return max(lo, min(hi, v))


class KeyReader(threading.Thread):
    """Doc phim o che do raw (non-blocking) trong mot luong rieng."""

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
            termios.tcsetattr(fd, termios.TCSADRAIN, old_attrs)


def main():
    parser = argparse.ArgumentParser(
        description='Ban phim -> /cmd_vel & /motor_enable (ROS 2 Jazzy, micro-ROS)')
    parser.add_argument('--step', type=float, default=DEFAULT_LINEAR_STEP,
                        help='Buoc tien/lui moi phim (m/s). Mac dinh: %(default)s')
    parser.add_argument('--astep', type=float, default=DEFAULT_ANGULAR_STEP,
                        help='Buoc quay moi phim (rad/s). Mac dinh: %(default)s')
    parser.add_argument('--max', type=float, default=DEFAULT_MAX_LINEAR,
                        help='Gioi han |linear| (m/s). Mac dinh: %(default)s')
    parser.add_argument('--amax', type=float, default=DEFAULT_MAX_ANGULAR,
                        help='Gioi han |angular| (rad/s). Mac dinh: %(default)s')
    parser.add_argument('--rate', type=float, default=DEFAULT_RATE_HZ,
                        help='Tan so publish (Hz). Mac dinh: %(default)s')
    parser.add_argument('--idle', type=float, default=DEFAULT_IDLE_STOP_S,
                        help='Auto-dung sau N giay khong nhan (0=tat deadman). Mac dinh: %(default)s')
    parser.add_argument('--topic', type=str, default=DEFAULT_CMD_TOPIC,
                        help='Topic cmd_vel. Mac dinh: %(default)s')
    parser.add_argument('--enable-topic', type=str, default=DEFAULT_ENABLE_TOPIC,
                        help='Topic motor_enable. Mac dinh: %(default)s')
    args = parser.parse_args()

    if not sys.stdin.isatty():
        print('Loi: can chay trong terminal (tty) de doc phim.', file=sys.stderr)
        sys.exit(1)

    rclpy.init()
    node = Node('teleop_key')
    pub_cmd = node.create_publisher(Twist, args.topic, 10)
    pub_enable = node.create_publisher(Bool, args.enable_topic, 10)

    print(HELP_TEXT.format(idle=args.idle))
    print("[1/2] Dang cho ket noi voi STM32 (DDS Discovery)...")
    sys.stdout.flush()

    # Cho cho den khi Publisher tim thay Subscriber tren STM32
    while rclpy.ok() and pub_cmd.get_subscription_count() == 0:
        rclpy.spin_once(node, timeout_sec=0.1)
        time.sleep(0.1)

    print("\n[2/2] ==> DA KET NOI VOI STM32! Nhan phiem 'e' de ENABLE dong co, W/S/A/D de chay.\n")
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
    motor_enabled = False

    twist = Twist()
    bool_msg = Bool()
    period = 1.0 / args.rate if args.rate > 0 else 0.05

    def publish_zero():
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        pub_cmd.publish(twist)

    def set_motor_enable(state: bool):
        nonlocal motor_enabled
        motor_enabled = state
        bool_msg.data = state
        pub_enable.publish(bool_msg)

    def print_state(extra_msg=""):
        status_str = "ENABLED" if motor_enabled else "DISABLED (press 'e')"
        sys.stdout.write(
            f'\r>> v={linear:+.2f} m/s  w={angular:+.2f} rad/s  [step={step:.2f} m/s]  [{status_str}] {extra_msg}   ')
        sys.stdout.flush()

    try:
        while running and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0)
            now = time.monotonic()
            changed = False
            status_msg = ""

            while not key_q.empty():
                try:
                    k = key_q.get_nowait()
                except queue.Empty:
                    break
                last_key_time = now
                if k in ('q', 'Q', '\x03'):
                    running = False
                    break
                elif k == ' ':
                    linear = 0.0
                    angular = 0.0
                    changed = True
                elif k in ('e', 'E'):
                    set_motor_enable(True)
                    status_msg = "[Motor ENABLED]"
                    changed = True
                elif k in ('x', 'X'):
                    set_motor_enable(False)
                    linear = 0.0
                    angular = 0.0
                    status_msg = "[Motor DISABLED]"
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
                    step = round(max(0.01, step / 1.25), 3)
                    astep = round(max(0.01, astep / 1.25), 3)
                    changed = True

            # Deadman (kich hoat neu --idle > 0)
            if args.idle > 0 and (now - last_key_time) > args.idle:
                if linear != 0.0 or angular != 0.0:
                    linear = 0.0
                    angular = 0.0
                    changed = True

            # Gioi han van toc an toan
            linear = round(clamp(linear, -args.max, args.max), 3)
            angular = round(clamp(angular, -args.amax, args.amax), 3)

            # Publish cmd_vel khi co thay doi hoac dang co van toc
            if changed or (linear != 0.0 or angular != 0.0):
                twist.linear.x = float(linear)
                twist.angular.z = float(angular)
                pub_cmd.publish(twist)

            if changed:
                print_state(status_msg)

            time.sleep(period)

    except KeyboardInterrupt:
        pass
    finally:
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
