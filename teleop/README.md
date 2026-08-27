# teleop_key — Điều khiển robot bằng bàn phím (ROS 2 Jazzy)

Script `teleop_key.py` publish `geometry_msgs/Twist` lên topic `/cmd_vel` và `std_msgs/Bool` lên topic `/motor_enable`.
STM32 (`stm32h7_node`) subscribe `/cmd_vel` và chuyển đổi sang vận tốc cho 2 động cơ RobStride qua động học vi sai (Kinematics).

> Chạy độc lập bằng `python3`, không cần colcon build. Chỉ cần ROS 2 Jazzy
> (`rclpy`, `geometry_msgs`, `std_msgs`) và micro-ROS Agent đang chạy.

---

## 🚀 Trình tự chạy (mở 3 terminal)

Mỗi terminal đều cần `source` môi trường ROS 2 trước khi chạy:

### ➊ Terminal 1 — micro-ROS Agent (hoặc qua systemd service)

Nếu chưa cài systemd service tự động:
```bash
source /opt/ros/jazzy/setup.bash
# Cổng USB CDC tự nhận diện qua symlink udev:
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/stm32_robot
# Hoặc cổng trực tiếp:
# ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
```

> **Lưu ý:** Giao thức USB CDC Virtual COM Port không phụ thuộc baudrate (bỏ qua cờ `-b 115200`).

### ➋ Terminal 2 — Kiểm tra STM32 node

```bash
source /opt/ros/jazzy/setup.bash
ros2 node list     # Phải thấy /stm32h7_node
ros2 topic list    # Phải thấy /cmd_vel, /motor_enable, /motor_fb, /motor_status
```

### ➌ Terminal 3 — Chạy teleop & Enable động cơ

```bash
source /opt/ros/jazzy/setup.bash
python3 teleop/teleop_key.py
```

> **Quan trọng:** Khi vừa kết nối, động cơ ở trạng thái an toàn (DISABLED).
> Nhấn phím **`e`** để ENABLE động cơ trước khi điều khiển bằng `w` `a` `s` `d`.

---

## 🎮 Bảng phím điều khiển

| Phím         | Tác dụng                                                 |
|--------------|----------------------------------------------------------|
| `e` / `E`    | **ENABLE động cơ** (gửi `/motor_enable` = True)         |
| `x` / `X`    | **DISABLE động cơ** (gửi `/motor_enable` = False)        |
| `w` / `W`    | Tiến thẳng (`linear.x += step`)                         |
| `s` / `S`    | Lùi (`linear.x -= step`)                                |
| `a` / `A`    | Quay trái (`angular.z += astep`)                        |
| `d` / `D`    | Quay phải (`angular.z -= astep`)                        |
| `space`      | **DỪNG NGAY** (linear=0, angular=0)                     |
| `+` / `=`    | Tăng bước vận tốc (×1.25)                                |
| `-` / `_`    | Giảm bước vận tốc (÷1.25)                                |
| `q` / Ctrl-C | Thoát (tự động gửi lệnh dừng trước khi thoát)            |

---

## ⚙️ Tham số dòng lệnh

| Tham số          | Ý nghĩa                                         | Mặc định | Đơn vị |
|------------------|-------------------------------------------------|----------|--------|
| `--step`         | Bước tiến/lùi mỗi phím                          | `0.10`   | m/s    |
| `--astep`        | Bước quay mỗi phím                              | `0.10`   | rad/s  |
| `--max`          | Giới hạn vận tốc dài \|linear\|                 | `0.80`   | m/s    |
| `--amax`         | Giới hạn vận tốc góc \|angular\|                | `1.50`   | rad/s  |
| `--rate`         | Tần số publish `/cmd_vel`                       | `20.0`   | Hz     |
| `--idle`         | Deadman: tự dừng sau N giây không nhấn phím     | `0.5`    | giây   |
| `--topic`        | Tên topic velocity                              | `/cmd_vel` | string |
| `--enable-topic` | Tên topic enable                                | `/motor_enable` | string |

Ví dụ:
```bash
# Tắt deadman (robot giữ nguyên vận tốc cho đến khi nhấn space)
python3 teleop/teleop_key.py --idle 0

# Điều chỉnh dải vận tốc cao hơn
python3 teleop/teleop_key.py --step 0.20 --max 1.20
```

---

## ⚠️ Lưu ý an toàn

- **Deadman mặc định (0.5s):** Nếu người dùng buông tay khỏi bàn phím quá 0.5s, script sẽ tự gửi lệnh dừng 0 m/s để chống trôi robot.
- **Dừng khẩn cấp:** Luôn sẵn sàng nhấn phím **`space`** hoặc **`x`** (disable) để ngắt chuyển động ngay lập tức.
