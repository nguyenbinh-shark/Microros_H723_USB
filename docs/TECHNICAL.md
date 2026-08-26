# BÁO CÁO KỸ THUẬT
## Robot Vi Sai micro-ROS trên STM32H723VGT6

**Ngày cập nhật:** 07/06/2026

---

## 1. Tổng quan

Báo cáo trình bày thiết kế và triển khai hệ thống nhúng điều khiển **robot bánh xe vi sai (differential drive)** sử dụng vi điều khiển STM32H723VGT6 tích hợp micro-ROS Jazzy. Board nhúng đóng vai trò một **ROS 2 node** hoàn chỉnh: đọc IMU, ước lượng vận tốc bằng Kalman filter, điều khiển 2 motor CyberGear qua FDCAN, và publish odometry cho Nav2.

---

## 2. Phần cứng

| Thành phần | Thông số |
|---|---|
| Vi điều khiển | STM32H723VGT6 (ARM Cortex-M7, 550 MHz) |
| Flash / RAM | 1024 KB / 560 KB (128 KB DTCMRAM + 320 KB AXI SRAM) |
| IMU | BMI088 — Accel ±6g + Gyro ±2000°/s, SPI2, 2 kHz ODR |
| Motor | 2× CyberGear BLDC (FDCAN protocol) |
| Motor ID 1 | FDCAN1 — bánh trái |
| Motor ID 2 | FDCAN2 — bánh phải (lắp đối xứng → sign đảo) |
| Giao tiếp PC | UART7 + USB-UART CH340, 115200 bps |
| Linker | `.bss` + heap chuyển sang AXI SRAM (vượt DTCMRAM 128 KB) |

---

## 3. Kiến trúc phần mềm

### 3.1 FreeRTOS Tasks

```
┌────────────────────────────────────────────────────────────────┐
│                        STM32H723VGT6                           │
│                                                                │
│  [main.c]  DWT_Init(550) → BMI088_init() → osKernelStart()     │
│                                                                │
│  ┌────────────┐ ┌────────────┐ ┌───────────┐ ┌─────────────┐   │
│  │ INS_Task   │ │OBSERVE_Task│ │MOTOR_Task │ │ defaultTask │   │
│  │ Realtime   │ │ High       │ │AboveNormal│ │ Normal      │   │
│  │ 1 ms/loop  │ │ 3 ms/loop  │ │10 ms/loop │ │ ~20 ms      │   │
│  │            │ │            │ │           │ │             │   │
│  │ BMI088 SPI │ │Kalman fuse │ │Diff-drive │ │ micro-ROS   │   │
│  │ Mahony AHRS│ │encoder+IMU │ │kinematics │ │ pub/sub     │   │
│  │ →quaternion│ │→v_filter   │ │→MIT mode  │ │ →/odom      │   │
│  └──────┬─────┘ └─────┬──────┘ └─────┬─────┘ └──────┬──────┘   │
│         │             │              │               │         │
│  SPI2   │   INS.ins_flag=1   FDCAN1/2 MIT    UART7-DMA Circular│
└────────────────────────────────────────────────────────────────┘
                                                        │
                                         115200 bps (CH340 USB)
                                                        │
┌───────────────────────────────────────────────────────▼────────┐
│                  Raspberry Pi 5 / PC (Ubuntu 24.04)            │
│                                                                │
│  micro_ros_agent ←→ ROS 2 Jazzy DDS                            │
│                                                                │
│  /imu  /euler  /motor_fb  /odom  ←→  /cmd_vel                  │
└────────────────────────────────────────────────────────────────┘
```

| Task | Stack | Priority | Period |
|---|---|---|---|
| INS_Task | 512 words (2 KB) | Realtime | 1 ms |
| OBSERVE_Task | 512 words (2 KB) | High | 3 ms |
| MOTOR_Task | 512 words (2 KB) | AboveNormal | 10 ms |
| defaultTask | 3000 words (12 KB) | Normal | ~20 ms |

> **Lưu ý:** micro-ROS yêu cầu stack lớn (≥ 8 KB) do nhiều lớp: RCL → RCLC → rmw → uxr client → UART transport.

### 3.2 Inter-task communication

- `INS_t INS` — global struct, viết bởi INS_Task, đọc bởi OBSERVE_Task và defaultTask
- `Observe_t Observe` — global struct (v_filter, omega_filter, x_filter), viết bởi OBSERVE_Task, đọc bởi defaultTask
- `RobotCmd_t g_robot_cmd` — protected bởi `g_cmd_mutex`, viết bởi cmd_vel_callback (ISR context), đọc bởi MOTOR_Task

---

## 4. INS Task — Mahony AHRS

BMI088 đọc ở 1 kHz. Thuật toán Mahony AHRS tích hợp gyro + accel để tính quaternion `INS.q[0..3]` và góc Euler.

```
Mahony filter: q += 0.5 × Ω_corrected × q × dt
               Ω_corrected = Ω_gyro + Kp×e_acc + Ki×∫e_acc
```

- Không dùng Kalman cho IMU (QuaternionEKF.h có trong repo nhưng không gọi)
- `ins_flag = 1` sau 3 giây: filter đã hội tụ
- Thời gian tính: ~20 µs/vòng (DWT hardware cycle counter)

---

## 5. OBSERVE Task — Kalman Velocity Observer

### Mục tiêu

Ước lượng vận tốc thực của robot bằng cách fuse dữ liệu encoder bánh xe (dễ trượt) và gia tốc kế IMU (tích phân drift). Kết quả chống trượt bánh tốt hơn đọc encoder thuần túy.

### Kalman state vector

```
x = [v; a]   (vận tốc m/s, gia tốc m/s²)
```

### Mô hình

```
F = [1, dt; 0, 1]    (state transition)
H = I₂               (full state observed)
Q = diag(0.5, 0.5)   (process noise)
R = diag(100, 100)   (measurement noise — trusts model more than sensor)
```

### Đo lường

```c
v_odom = (CAN_GetMotorVel(1) × WHEEL_RADIUS  +
          CAN_GetMotorVel(2) × WHEEL_RADIUS) / 2.0f
a_imu  = INS.MotionAccel_b[0]   // forward acceleration body frame
```

### Output

```c
Observe.v_filter     = vel_acc[0]   // Kalman-fused velocity (m/s)
Observe.omega_filter = INS.Gyro[2]  // yaw rate (rad/s)
Observe.x_filter    += v_filter × dt  // integrated position (m)
```

---

## 6. MOTOR Task — Differential Drive

### Kinematics

```
v_left_rad  = (v − ω × L/2) / R × MOTOR_LEFT_SIGN
v_right_rad = (v + ω × L/2) / R × MOTOR_RIGHT_SIGN
```

Trong đó `v` (m/s), `ω` (rad/s) đến từ `/cmd_vel`, `R` = bán kính bánh, `L` = khoảng cách tâm 2 bánh.

### MIT velocity mode

```c
mit_ctrl(&hfdcan1, 1U, 0.0f, v_left_rad,  0.0f, MOTOR_VEL_KD, 0.0f);
mit_ctrl(&hfdcan2, 2U, 0.0f, v_right_rad, 0.0f, MOTOR_VEL_KD, 0.0f);
// kp=0, kd=Kd → torque = Kd × (v_target − v_actual)
```

### Thông số cơ học (chỉnh theo robot thực)

| Hằng số | File | Giá trị placeholder |
|---|---|---|
| `WHEEL_RADIUS` | motor_task.h | 0.05 m |
| `WHEEL_BASE` | motor_task.h | 0.30 m |
| `MOTOR_LEFT_SIGN` | motor_task.h | +1.0 |
| `MOTOR_RIGHT_SIGN` | motor_task.h | −1.0 |
| `MOTOR_VEL_KD` | motor_task.h | 1.0 |

---

## 7. micro-ROS — Topics & Node

### Topics

| Topic | Type | Hz | Nội dung |
|---|---|---|---|
| `/imu` | sensor_msgs/Imu | 50 | Quaternion · gyro · accel |
| `/euler` | geometry_msgs/Vector3 | 50 | Roll · Pitch · Yaw (độ) |
| `/motor_fb` | sensor_msgs/JointState | 50 | pos/vel motor ID 1 |
| `/odom` | nav_msgs/Odometry | 50 | Dead-reckoning pose + Kalman twist |
| `/cmd_vel` | geometry_msgs/Twist | — | Subscribe: linear.x, angular.z |

### /odom message

```
header.frame_id    = "odom"
child_frame_id     = "base_link"

pose.pose.position.x/y       ← dead-reckoning (Observe.v_filter × cos/sin(yaw) × dt)
pose.pose.orientation        ← quaternion INS.q[0..3]
twist.twist.linear.x         ← Observe.v_filter     (Kalman-filtered, slip-resistant)
twist.twist.angular.z        ← Observe.omega_filter  (gyro.z)
```

Node name: `stm32h7_node`

---

## 8. Cấu trúc file

```
User/APP/
├── INS_task.c/h       — Mahony AHRS, quaternion output
├── observe_task.c/h   — Kalman velocity observer
├── motor_task.c/h     — Diff-drive kinematics, CyberGear MIT mode
                         (WHEEL_RADIUS, WHEEL_BASE, MOTOR_SIGN — single source)
Core/Src/
└── freertos.c         — Task creation, micro-ROS publishers/subscriber
User/Bsp/
└── can_bsp.c/h        — CAN_GetMotorVel(id): track 2 motors independently
```

---

## 9. Môi trường phát triển

| Thành phần | Phiên bản |
|---|---|
| OS nhúng | FreeRTOS + CMSIS-RTOS V2 |
| micro-ROS | Jazzy Jalisco (libmicroros.a static) |
| Toolchain | arm-none-eabi-gcc, Makefile |
| PC | Windows 11 + WSL2 Ubuntu 24.04 |
| ROS 2 | Jazzy Jalisco |
| Transport | UART7 DMA Circular (TX + RX Very High priority) |

---

## 10. Quy trình build & flash

```bash
# Build (WSL2)
wsl -d Ubuntu-24.04
cd /mnt/p/Prj_STM32/ros_h7
make -j$(nproc) 2>&1 | tail -5

# Flash: STM32CubeProgrammer hoặc OpenOCD qua ST-Link
```

---

## 11. Khởi động Agent

```bash
# Forward USB vào WSL2 (Windows PowerShell Admin)
usbipd attach --wsl --busid <busid>

# WSL2
sudo chmod 666 /dev/ttyUSB0
source ~/microros_ws/install/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
```

Chờ log `[3/3] Session established!` → board kết nối thành công.

```bash
# Kiểm tra
ros2 topic list        # phải thấy: /imu /euler /motor_fb /odom /cmd_vel
ros2 topic hz /odom    # phải ≈ 50 Hz
ros2 topic echo /odom  # kiểm tra twist.twist.linear.x khi robot di chuyển
```
