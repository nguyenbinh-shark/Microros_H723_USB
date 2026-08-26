# BÁO CÁO KỸ THUẬT
## Robot Vi Sai micro-ROS trên STM32H723VGT6 (USB CDC)

**Ngày cập nhật:** 26/08/2026

---

## 1. Tổng quan

Báo cáo trình bày thiết kế và triển khai hệ thống nhúng điều khiển **robot bánh xe vi sai (differential drive)** sử dụng vi điều khiển STM32H723VGT6 tích hợp micro-ROS Jazzy. Board nhúng đóng vai trò một **ROS 2 node** hoàn chỉnh:
- Giao tiếp với ROS 2 qua **USB CDC tốc độ cao (12 Mbps)**.
- Xuất log chẩn đoán qua **UART7 (PE8 @ 115200 bps)**.
- Đọc IMU 6-DOF **BMI088 qua SPI2**, tính toán góc Euler và Quaternion bằng **Mahony AHRS**.
- Ước lượng vận tốc thực và bù trượt bánh bằng **Kalman Velocity Observer**.
- Điều khiển 2 motor không chổi than **RobStride qua FDCAN1 và FDCAN3**.

---

## 2. Phần cứng

| Thành phần | Thông số |
|---|---|
| Vi điều khiển | STM32H723VGT6 (ARM Cortex-M7, 550 MHz) |
| Flash / RAM | 1024 KB / 560 KB (128 KB DTCMRAM + 320 KB AXI SRAM) |
| IMU | BMI088 — Accel ±6g + Gyro ±2000°/s, SPI2, 1 kHz ODR |
| Motor | 2× RobStride BLDC (FDCAN protocol) |
| Motor ID 1 | FDCAN1 — bánh trái |
| Motor ID 2 | FDCAN3 — bánh phải |
| Giao tiếp PC (micro-ROS) | USB OTG HS (Internal FS PHY) → Virtual COM Port (`/dev/ttyACM0`), 12 Mbps |
| Giao tiếp Debug Log | UART7 (PE8 TX) → USB-TTL, 115200 bps 8N1 |

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
│  │ →quaternion│ │→v_filter   │ │→RobStride │ │ (USB CDC)   │   │
│  └──────┬─────┘ └─────┬──────┘ └─────┬─────┘ └──────┬──────┘   │
│         │             │              │               │         │
│  SPI2   │   INS.ins_flag=1   FDCAN1/3 CAN      USB CDC Transport
└──────────────────────────────────────────────────────┼─────────┘
                                                       │
                                        12 Mbps USB Type-C Cable
                                                       │
┌──────────────────────────────────────────────────────▼─────────┐
│                  Raspberry Pi 5 / PC (Ubuntu 24.04)            │
│                                                                │
│  micro_ros_agent (/dev/ttyACM0) ←→ ROS 2 Jazzy DDS            │
│                                                                │
│  /motor_fb  /imu  /odom  ←→  /cmd_vel  /motor_enable           │
└────────────────────────────────────────────────────────────────┘
```

| Task | Stack | Priority | Chu kỳ | Chức năng |
|---|---|---|---|---|
| `INS_Task` | 2 KB | Realtime | 1 ms | Đọc BMI088, chạy Mahony AHRS → quaternion, Euler |
| `OBSERVE_Task` | 2 KB | High | 3 ms | Kalman filter: fuse encoder + IMU accel → `v_filter` |
| `MOTOR_Task` | 2 KB | AboveNormal | 10 ms | Diff-drive kinematics → Điều khiển RobStride FDCAN |
| `defaultTask` | 12 KB | Normal | ~20 ms | micro-ROS: publish feedback, nhận `/cmd_vel` |

---

## 4. Giao thức Truyền thông & micro-ROS Topics

| Topic | Type | Hướng | Mô tả |
|---|---|:---:|---|
| `/cmd_vel` | `geometry_msgs/Twist` | **Sub** | Nhận vận tốc dài `linear.x` và góc `angular.z` |
| `/motor_enable` | `std_msgs/Bool` | **Sub** | Kích hoạt hoặc ngắt motor an toàn |
| `/motor_fb` | `sensor_msgs/JointState` | **Pub** | Góc, vận tốc và moment tải 2 bánh |
| `/imu` | `sensor_msgs/Imu` | **Pub** | Quaternion, gia tốc và vận tốc góc |
| `/odom` | `nav_msgs/Odometry` | **Pub** | Tọa độ vị trí và vận tốc xe phục vụ Nav2 |

---

## 5. Cấu trúc mã nguồn

```
Core/
├── Inc/               — Header cấu hình ngoại vi & FreeRTOSConfig.h
└── Src/               — main.c, freertos.c, usart.c, fdcan.c, usb_device.c
USB_DEVICE/            — Stack USB Device CDC của ST
User/
├── APP/               — INS_task.c, observe_task.c, motor_task.c
├── Algorithm/         — Mahony filter, Kalman filter, Kinematics
├── Bsp/               — can_bsp.c, bsp_dwt.c, bsp_PWM.c
└── Devices/           — RobStride driver, BMI088 driver
micro_ros_stm32cubemx_utils/
└── extra_sources/
    └── microros_transports/
        └── usb_cdc_transport.c — Custom transport micro-ROS qua USB CDC
```
