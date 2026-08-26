# teleop_key — Điều khiển robot bằng bàn phím (ROS 2 Jazzy)

Script `teleop_key.py` publish `geometry_msgs/Twist` lên topic `/cmd_vel`.
STM32 (firmware `ros_h7`) subscribe `/cmd_vel` rồi điều khiển 2 động cơ.

> Chạy độc lập bằng `python3`, không cần build package. Chỉ cần ROS 2 Jazzy
> (`rclpy`, `geometry_msgs`) và micro-ROS agent đang chạy.

---

## 🚀 Trình tự chạy (mở 3 terminal)

Mỗi terminal đều cần `source` ROS 2 trước. Làm theo thứ tự ➊ → ➋ → ➌.

### ➊ Terminal 1 — micro-ROS agent (chạy liên tục, đừng tắt)

```bash
source /opt/ros/jazzy/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
```

> Bridge PC ↔ STM32 qua UART. Thay `--dev` / `-b` cho đúng cổng và baudrate đã cấu hình trong firmware.

### ➋ Terminal 2 — kiểm tra STM32 đã lên

```bash
source /opt/ros/jazzy/setup.bash
ros2 node list     # phải thấy /stm32h7_node
ros2 topic list    # phải thấy /cmd_vel
```

> Chưa thấy node? Kiểm tra agent đã kết nối UART và firmware STM32 đã nạp bản mới nhất.

### ➌ Terminal 3 — chạy teleop

```bash
source /opt/ros/jazzy/setup.bash
python3 teleop/teleop_key.py          # dùng w a s d để điều khiển
```

---

## Tham số tùy chọn (mặc định đã an toàn)

| Tham số   | Ý nghĩa                                  | Mặc định |
|-----------|------------------------------------------|----------|
| `--step`  | Bước tiến/lùi mỗi phím (Nm)              | 0.05     |
| `--astep` | Bước quay mỗi phím (Nm)                  | 0.05     |
| `--max`   | Giới hạn \|linear\| (Nm)                 | 0.5      |
| `--amax`  | Giới hạn \|angular\| (Nm)                | 0.5      |
| `--rate`  | Tần số publish /cmd_vel (Hz)             | 20       |
| `--idle`  | Auto-dừng sau N giây không nhấn (deadman)| 0.5      |
| `--topic` | Topic xuất                              | /cmd_vel |

Ví dụ tăng lực: `python3 teleop/teleop_key.py --step 0.1 --max 1.0`

## Bảng phím

| Phím        | Tác dụng                         |
|-------------|----------------------------------|
| `w` / `W`   | Tiến thẳng (linear += step)      |
| `s` / `S`   | Lùi (linear -= step)             |
| `a` / `A`   | Quay trái (angular += step)      |
| `d` / `D`   | Quay phải (angular -= step)      |
| `space`     | **DỪNG ngay (zero)**             |
| `+` / `=`   | Tăng bước step (×1.25)           |
| `-` / `_`   | Giảm bước step (÷1.25)           |
| `q` / Ctrl-C| Thoát (gửi zero trước khi thoát) |

## ⚠️ An toàn (đọc trước khi chạy)

Firmware hiện tại (`User/APP/motor_task.c`) dùng `linear.x` / `angular.z` **trực tiếp làm moment lực (Nm)** — tức **điều khiển moment hở** (lỗi #4 chưa sửa). Ghi nhớ:

- **`space`** → dừng khẩn ngay (zero).
- **`q` / Ctrl-C** → thoát; script luôn gửi `0` trước → STM32 disable motor.
- **Deadman** (`--idle`, mặc định `0.5`s): không nhấn trong 0.5s → tự gửi `0`. Tắt bằng `--idle 0` (**cẩn thận**).
- **Bắt đầu nhỏ**: giữ `--step 0.05`; tăng dần sau khi thấy động cơ phản hồi đúng chiều/lực.
