# STM32H723 micro-ROS Node — Hướng dẫn sử dụng

Board STM32H723VGT6 chạy FreeRTOS + micro-ROS Jazzy. Hoạt động như một **ROS 2 node robot vi sai (differential drive)**: publish IMU, odometry, phản hồi motor; subscribe lệnh điều khiển `/cmd_vel`. Tích hợp Kalman filter ước lượng vận tốc để bù trượt bánh.

> 📚 **Tài liệu kỹ thuật chi tiết:** Xem [docs/TECHNICAL.md](docs/TECHNICAL.md)  
> 🔧 **Hướng dẫn gỡ lỗi (debug) với VS Code:** Xem [docs/DEBUG.md](docs/DEBUG.md)  
> 📖 **Manual motor CyberGear:** Xem [docs/reference/cybergear_motor.md](docs/reference/cybergear_motor.md)

---

## Phần cứng

| Thành phần | Chi tiết |
|---|---|
| Vi điều khiển | STM32H723VGT6 @ 550 MHz |
| IMU | BMI088 (SPI2) — accel + gyro |
| Motor | 2× CyberGear FDCAN (ID 1 = trái/FDCAN1, ID 2 = phải/FDCAN2) |
| Giao tiếp PC | UART7 → USB-UART CH340 (`/dev/ttyUSB0`) |
| Baud rate | 115200 bps |

---

## Kiến trúc phần mềm (FreeRTOS tasks)

| Task | Priority | Period | Chức năng |
|---|---|---|---|
| `INS_Task` | Realtime | 1 ms | Đọc BMI088, chạy Mahony AHRS → quaternion |
| `OBSERVE_Task` | High | 3 ms | Kalman filter: fuse encoder + IMU accel → `v_filter`, `omega_filter` |
| `MOTOR_Task` | AboveNormal | 10 ms | Diff-drive kinematics → MIT mode CyberGear |
| `defaultTask` | Normal | ~20 ms | micro-ROS: publish 4 topics, spin executor |

---

## ROS 2 Topics

| Topic | Type | Chiều | Mô tả |
|---|---|---|---|
| `/imu` | `sensor_msgs/Imu` | Publish 50Hz | Quaternion · gyro (rad/s) · accel (m/s²) |
| `/euler` | `geometry_msgs/Vector3` | Publish 50Hz | Roll · Pitch · Yaw (đơn vị: **độ**) |
| `/motor_fb` | `sensor_msgs/JointState` | Publish 50Hz | Vị trí (rad) · vận tốc (rad/s) motor ID 1 |
| `/odom` | `nav_msgs/Odometry` | Publish 50Hz | Pose (dead-reckoning) · twist (Kalman-filtered) |
| `/cmd_vel` | `geometry_msgs/Twist` | Subscribe | `linear.x` (m/s) · `angular.z` (rad/s) |

Node name: `stm32h7_node`. Frame: `odom` → `base_link`.

---

## Cài đặt micro-ROS Agent (chỉ làm 1 lần)

Yêu cầu: ROS 2 Jazzy đã cài trên Ubuntu 24.04.

```bash
mkdir -p ~/microros_ws/src && cd ~/microros_ws
git clone -b jazzy https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

sudo apt update && rosdep update
rosdep install --from-paths src --ignore-src -y

colcon build
source install/local_setup.bash

ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash
```

---

## 🎯 Khởi động hệ thống (mỗi lần dùng)

### Bước 1 — Kết nối board

Cắm cáp CH340 từ board vào PC. Board tự khởi động, **không cần giữ phẳng** (dùng hardcoded offset, không calibrate).

### Bước 2 — Forward USB vào WSL2 (Windows PowerShell — Admin)

```powershell
usbipd list                        # tìm CH340, ví dụ busid 1-2
usbipd attach --wsl --busid 1-2
```

### Bước 3 — Chạy micro-ROS Agent (WSL2 Ubuntu)

```bash
wsl -d Ubuntu-24.04

sudo chmod 666 /dev/ttyUSB0
source ~/microros_ws/install/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
```

Chờ agent log `[3/3] Session established!` là board đã kết nối.

---

## 🖥️ Bảng lệnh Terminal thường dùng

### 📌 Bảng tổng quan nhanh

| Bước | Mục đích | Môi trường thực thi | Cần quyền Admin? |
|:----:|----------|---------------------|:----------------:|
| **a** | Biên dịch firmware | WSL/Ubuntu | ❌ |
| **b** | Chạy micro-ROS Agent | WSL/Ubuntu (terminal #1) | ❌ (dùng `sudo`) |
| **c** | Tương tác robot qua ROS 2 | WSL/Ubuntu (terminal #2) | ❌ |
| **d** | Chuyển tiếp USB vào WSL2 | Windows PowerShell | ✅ (Admin) |

> 💡 **Quy trình tham khảo:** Làm theo thứ tự **d → b → c**. Bước `d` (chuyển tiếp USB) cần thực hiện **trước** để WSL nhìn thấy board STM32.

---

### a. Biên dịch Firmware

> **Môi trường:** WSL/Ubuntu · Thư mục: `/mnt/p/Prj_STM32/ros_h7`

Xóa các file build cũ (chỉ khi cần làm sạch hoàn toàn):
```bash
make clean
```

Biên dịch code:
```bash
make -j$(nproc)
```

---

### b. Chạy micro-ROS Agent

> **Môi trường:** WSL/Ubuntu · Dùng **terminal #1**

**1.** Cấp quyền cho cổng serial (tên cổng có thể là `ttyACM0` hoặc `ttyUSB0`):
```bash
sudo chmod 666 /dev/ttyACM0
```

**2.** Kích hoạt môi trường ROS 2:
```bash
source ~/microros_ws/install/setup.bash
```

**3.** Khởi động Agent:
```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 921600
```

> ⚠️ **Lưu ý:** Sau khi chạy lệnh này, cần **nhấn nút RESET** trên board STM32 để thiết lập kết nối.

---

### c. Tương tác với Robot qua ROS 2

> **Môi trường:** WSL/Ubuntu · Dùng **terminal #2** (đã chạy lệnh `source` ở phần **b**)

**Liệt kê tất cả các topic:**
```bash
ros2 topic list
```

**Kiểm tra tần số của một topic** (ví dụ: `/odom`):
```bash
ros2 topic hz /odom
```

**Xem dữ liệu của một topic** (ví dụ: `/odom` hoặc `/imu`):
```bash
ros2 topic echo /odom
```

**Gửi lệnh điều khiển:**

| Hành động | Lệnh |
|-----------|------|
| Chạy thẳng | `ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}" --once` |
| Xoay tại chỗ | `ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.1}}" --once` |
| Dừng robot | `ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" --once` |

---

### d. Quản lý thiết bị USB (Windows PowerShell)

> **Môi trường:** Windows PowerShell · **Yêu cầu quyền Admin**
>
> Các lệnh này dùng để "chuyển tiếp" (forward) thiết bị USB từ Windows vào môi trường WSL2.

**Liệt kê các thiết bị USB** (để tìm **BUSID** của thiết bị USB-to-Serial, ví dụ CH340):
```powershell
usbipd list
```

**Đính kèm (attach)** thiết bị vào WSL — thay `1-3` / `1-1` bằng BUSID thực tế vừa tìm được:
```powershell
usbipd attach --wsl --busid 1-3
usbipd attach --wsl --busid 1-1
```

**Gỡ (detach)** thiết bị khỏi WSL — dùng khi muốn ngắt kết nối hoặc khi thiết bị bị "kẹt":
```powershell
usbipd detach --busid 1-3
usbipd detach --busid 1-1
```

---

## Đọc odometry (Nav2)

```bash
ros2 topic echo /odom
```

Trường quan trọng:
- `pose.pose.position.x/y` — vị trí dead-reckoning (m)
- `pose.pose.orientation` — quaternion từ Mahony AHRS
- `twist.twist.linear.x` — vận tốc Kalman-filtered (slip-resistant)
- `twist.twist.angular.z` — yaw rate từ gyro

---

## Đọc dữ liệu IMU

Mở terminal WSL thứ 2:

```bash
source ~/microros_ws/install/setup.bash

# Góc Euler (độ) — ổn định sau ~3 giây
ros2 topic echo /euler

# Dữ liệu IMU đầy đủ (quaternion + gyro + accel)
ros2 topic echo /imu

# Tần số publish thực tế
ros2 topic hz /euler
```

**Giá trị bình thường khi board đặt phẳng:**
- `/euler`: x ≈ 0°, y ≈ 0°, z = yaw tùy hướng
- `/imu` angular_velocity: |x|, |y|, |z| < 0.05 rad/s
- `/imu` linear_acceleration: z ≈ 9.81 m/s²

> **Lưu ý:** Góc Euler chỉ hợp lệ sau 3 giây khởi động (`ins_flag = 1`). Trước đó giá trị bằng 0.

---

## Đọc phản hồi motor

```bash
ros2 topic echo /motor_fb
```

```
name: [motor_1]
position: [1.234]   # rad
velocity: [0.567]   # rad/s
effort:   [0.0]
```

---

## Gửi lệnh điều khiển motor

```bash
# Tiến thẳng 0.5 m/s
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5}, angular: {z: 0.0}}" --once

# Quay tại chỗ
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 1.0}}" --once

# Dừng (disable motor)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}" --once
```

Motor **enable** khi `linear.x != 0` hoặc `angular.z != 0`. Motor **disable** khi cả hai bằng 0.

---

## Dùng từ máy ROS 2 khác trên cùng mạng LAN

Không cần cài thêm gì. ROS 2 dùng DDS multicast tự động discover.

**Yêu cầu:**
- Cùng `ROS_DOMAIN_ID` (mặc định = 0)
- Cùng mạng LAN, firewall cho phép UDP multicast
- Cùng phiên bản ROS 2 Jazzy

**Máy PC chủ (chạy agent):**
```bash
export ROS_DOMAIN_ID=0        # hoặc thêm vào ~/.bashrc
source ~/microros_ws/install/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
```



**Máy khác (cùng LAN):**
```bash
export ROS_DOMAIN_ID=0
source /opt/ros/jazzy/setup.bash

ros2 topic list               # phải thấy /imu, /euler, /motor_fb, /cmd_vel
ros2 topic echo /euler
```

Nếu không thấy topic, thử tắt tường lửa hoặc thêm rule cho UDP port 7400–7500:
```bash
sudo ufw allow 7400:7500/udp
```

---

## Build firmware (nếu cần sửa code)

```bash
wsl -d Ubuntu-24.04
cd /mnt/p/Prj_STM32/ros_h7
make -j$(nproc) 2>&1 | tail -5
```

Flash bằng STM32CubeProgrammer hoặc OpenOCD qua ST-Link.

---

## Troubleshooting

| Vấn đề | Nguyên nhân | Giải pháp |
|---|---|---|
| Agent không kết nối được | Sai cổng serial | Kiểm tra `ls /dev/ttyUSB*`, thử `ttyUSB1` |
| `/dev/ttyUSB0` không xuất hiện trong WSL | Chưa attach USB | Chạy `usbipd attach --wsl --busid X-X` từ Windows |
| `/euler` nhảy loạn trong 3s đầu | Bình thường, filter chưa hội tụ | Chờ `ins_flag=1` sau ~3 giây |
| Máy khác không thấy topic | DDS không discover | Kiểm tra cùng `ROS_DOMAIN_ID`, mở UDP firewall |
| Motor không chạy | `cmd_vel` enable=0 | Gửi `linear.x != 0` hoặc `angular.z != 0` |
| `/odom` position drift | Trượt bánh hoặc sai WHEEL_RADIUS | Chỉnh `WHEEL_RADIUS`/`WHEEL_BASE` trong [User/APP/motor_task.h](User/APP/motor_task.h) |
| `v_filter` = 0 mãi | Observe_task chưa nhận feedback CAN | Kiểm tra FDCAN1/2 wiring và ID motor |

---

## Thông số cơ học (cần đo thực tế)

Các hằng số trong [User/APP/motor_task.h](User/APP/motor_task.h):

| Hằng số | Giá trị mặc định | Ý nghĩa |
|---|---|---|
| `WHEEL_RADIUS` | 0.05 m | Bán kính bánh xe |
| `WHEEL_BASE` | 0.30 m | Khoảng cách tâm 2 bánh |
| `MOTOR_LEFT_SIGN` | +1.0 | Chiều quay motor ID 1 |
| `MOTOR_RIGHT_SIGN` | -1.0 | Chiều quay motor ID 2 (đối xứng) |
| `MOTOR_VEL_KD` | 1.0 | Hệ số Kd MIT velocity mode |

cd /mnt/p/Prj_STM32/ros_h7
source /opt/ros/jazzy/setup.bash
python3 teleop/teleop_key.py


