# STM32H723 micro-ROS Robot Controller (USB CDC)

Firmware điều khiển **Robot vi sai (Differential Drive)** chạy trên vi điều khiển **STM32H723VGT6 (192 MHz)** kết hợp **FreeRTOS** và **micro-ROS** giao tiếp qua **cổng USB CDC tốc độ cao (12 Mbps)**.

---

## 📌 Tính năng nổi bật
- **Giao tiếp micro-ROS qua USB CDC**: Truyền nhận trực tiếp với ROS 2 qua cổng USB Type-C của STM32 (nhận diện như `/dev/ttyACM0` trên Linux hoặc `COMx` trên Windows), băng thông lớn và độ trễ cực thấp.
- **Kênh Serial Debug độc lập**: Cổng **UART7 (chân PE8 @ 115200 bps)** xuất log `printf` thời gian thực (trạng thái boot, kết nối agent, lệnh `/cmd_vel`, phát hiện lỗi HardFault / tràn RAM).
- **Bộ lọc tư thế Mahony AHRS**: Đọc dữ liệu từ IMU **BMI088 (SPI2)** với tần số 1 kHz để tính toán góc nghiêng Roll, Pitch, Yaw và Quaternion.
- **Bộ lọc Kalman (Velocity Observer)**: Kết hợp vận tốc bánh xe và gia tốc IMU để ước lượng vận tốc thực tế, bù trượt bánh.
- **Màn hình LCD Debug (ST7789V 240x280 SPI1)**: Hiển thị trực quan Dashboard thời gian thực trên bo mạch (trạng thái kết nối micro-ROS Agent, vận tốc `/cmd_vel`, tốc độ & moment 2 bánh RobStride, góc nghiêng IMU Roll/Pitch/Yaw, bus CAN).
- **Điều khiển 2 Motor RobStride**: Điều khiển qua 2 bus độc lập **FDCAN1 (Bánh trái)** và **FDCAN3 (Bánh phải)** bằng thuật toán vòng kín FOC Stiffness Control.

---

## 🔌 Sơ đồ Chân & Kết nối Phần cứng

| Ngoại vi | Chân STM32 | Chức năng / Kết nối |
| :--- | :--- | :--- |
| **micro-ROS (USB CDC)** | **PA11 (DM), PA12 (DP)** | Cổng USB Type-C chính cắm vào máy tính / Raspberry Pi |
| **Debug Serial Log** | **PE8 (UART7_TX)** | Cắm vào chân **RX** của mạch USB-TTL (Baudrate: **115200**) |
| **Màn hình LCD (SPI1)** | **PB3 (SCK), PD7 (MOSI)**<br>**PE15 (CS), PD10 (DC), PB11 (RES), PB10 (BLK)** | Màn hình màu ST7789V hiển thị Dashboard gỡ lỗi |
| **Nút điều hướng 5 chiều (ADC1)** | **PA5 (ADC1_IN19)** | Nút gạt 5 chiều chuyển 4 trang hiển thị trên màn hình LCD |
| **Tay cầm RC SBUS/DBUS (UART5)** | **PD2 (UART5_RX)** | Cổng nhận tín hiệu tay cầm RC (100k 8E2, DMA1 Stream 4) |
| **IMU BMI088 (SPI2)** | **PB13 (SCK), PB14 (MISO), PB15 (MOSI)**<br>**PC4 (ACC_CS), PC5 (GYRO_CS)** | Cảm biến IMU 6-DOF trên board |
| **Motor Trái (FDCAN1)** | **PD0 (RX), PD1 (TX)** · **PC13 (EN)** | CAN Bus Motor 1 (Left), 1 Mbps |
| **Motor Phải (FDCAN3)** | **PD12 (RX), PD13 (TX)** · **PC15 (EN)** | CAN Bus Motor 2 (Right), 1 Mbps |

> Chân EN transceiver (PC13/PC14/PC15) là **active-HIGH** — phải kéo lên mức cao.
> Cấu hình tại `CAN_XCVR_NORMAL_MODE_LEVEL` trong [robot_config.h](User/Config/robot_config.h).

---

## 📡 Danh sách ROS 2 Topics

| Topic | Kiểu Message | Chiều | Tần số / Sự kiện | Mô tả |
| :--- | :--- | :---: | :---: | :--- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | **Sub** | Nhận tức thời | Vận tốc dài `linear.x` (m/s) và vận tốc góc `angular.z` (rad/s) |
| `/motor_enable` | `std_msgs/msg/Bool` | **Sub** | Sự kiện | `true`: Bật motor chạy; `false`: Ngắt motor (thả trôi/phanh) |
| `/motor_fb` | `sensor_msgs/msg/JointState` | **Pub** | 50 Hz | Phản hồi góc quay (rad), tốc độ (rad/s) và tải moment (Nm). QoS **BEST_EFFORT**, `frame_id` = `base_link` |
| `/motor_status` | `std_msgs/msg/UInt8` | **Pub** | 5 Hz | Bitmask tình trạng link CAN / motor — xem bảng dưới |

> **Node Name**: `stm32h7_node`  
> **Frame ID**: `base_link`

`/motor_fb` dùng QoS **BEST_EFFORT**. Subscriber mặc định RELIABLE sẽ không nhận được gì:

```bash
ros2 topic echo /motor_fb --qos-reliability best_effort
ros2 topic info /motor_fb -v          # đối chiếu QoS hai đầu khi nghi ngờ
```

### Bitmask `/motor_status`

| Bit | Giá trị | Ý nghĩa |
| :-: | :-- | :-- |
| 0 | `0x01` | Feedback bánh trái (FDCAN1) còn tươi |
| 1 | `0x02` | Feedback bánh phải (FDCAN3) còn tươi |
| 2 | `0x04` | FDCAN1 đang Bus_Off |
| 3 | `0x08` | FDCAN3 đang Bus_Off |
| 4 | `0x10` | Motor đang ở trạng thái enable |
| 5 | `0x20` | `/cmd_vel` timeout — xe đang bị giữ dừng |

Bit 0/1 giúp phân biệt "bánh đứng yên" (velocity = 0, bit = 1) với "mất CAN"
(velocity = 0, bit = 0) — điều mà riêng `/motor_fb` không diễn đạt được.

---

## 🚀 Hướng dẫn Sử dụng Nhanh (Copy-Paste Code)

### Bước 1: Cài đặt micro-ROS Agent trên Máy tính (Chỉ làm 1 lần)

#### Cách A: Cài đặt trực tiếp trên Ubuntu (Khuyên dùng)
```bash
# Tạo workspace cho micro-ROS Agent
mkdir -p ~/microros_ws/src && cd ~/microros_ws/src
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git

cd ~/microros_ws
sudo apt update && rosdep update
rosdep install --from-paths src --ignore-src -y
colcon build
source install/local_setup.bash

# Build riêng gói Agent
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash
```

#### Cách B: Chạy nhanh qua Docker (Không cần cài đặt gì thêm)
```bash
docker run -it --rm -v /dev:/dev --privileged --net=host microros/micro-ros-agent:$ROS_DISTRO serial --dev /dev/ttyACM0
```

---

### Bước 2: Kết nối Phần cứng & Khởi chạy

1. **Cắm cáp USB Type-C** từ cổng USB STM32 vào máy tính / Raspberry Pi.
2. *(Tùy chọn)* Cắm module USB-TTL vào chân **PE8 (TX)** và **GND** để xem log debug trên Serial Monitor (Baudrate: **115200**).
3. **Nếu dùng WSL2 trên Windows**, mở PowerShell (Admin) để chuyển tiếp cổng USB vào WSL:
   ```powershell
   usbipd list                          # Xem BUSID của thiết bị (VD: 1-2)
   usbipd attach --wsl --busid 1-2      # Forward vào WSL
   ```
4. **Chạy micro-ROS Agent trên Ubuntu / WSL2**:
   ```bash
   sudo chmod 666 /dev/ttyACM0
   source ~/microros_ws/install/setup.bash
   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
   ```
5. Khi kết nối thành công, terminal Agent sẽ hiển thị:
   ```text
   [1724687654.123] [micro_ros_agent] [INFO] Session established!
   ```

---

## 📋 Bảng Lệnh Điều khiển & Kiểm tra Topic (Copy-Paste)

Mở một Terminal mới (sau khi Agent đã chạy) và chạy các lệnh dưới đây:

### 1. Kiểm tra danh sách Node và Topic
```bash
# Xem danh sách các node đang online
ros2 node list

# Xem danh sách các topic
ros2 topic list
```

### 2. Xem dữ liệu phản hồi từ Motor
```bash
ros2 topic echo /motor_fb
```

### 3. Bật / Tắt Motor
```bash
# Bật Motor (Enable)
ros2 topic pub --once /motor_enable std_msgs/msg/Bool "{data: true}"

# Tắt Motor (Disable / Safe stop)
ros2 topic pub --once /motor_enable std_msgs/msg/Bool "{data: false}"
```

### 4. Gửi lệnh lái Robot di chuyển
```bash
# Chạy thẳng tới với vận tốc 0.2 m/s
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Xoay tại chỗ sang trái với tốc độ 0.5 rad/s
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"

# Dừng hẳn robot
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### 5. Lái robot bằng bàn phím (Teleop Keyboard)
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## 🛠️ Hướng dẫn Biên dịch Firmware (Build Code)

Dự án sử dụng file `Makefile` tiêu chuẩn với trình biên dịch `arm-none-eabi-gcc`.

### Trên Linux / WSL:
```bash
# Biên dịch toàn bộ firmware
make -j$(nproc)

# Xóa file build cũ
make clean
```

### Trên Windows PowerShell (sử dụng Toolchain STM32CubeIDE):
```powershell
$env:PATH = "C:\ST\STM32CubeIDE_1.16.1\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.12.3.rel1.win32_1.0.200.202406191623\tools\bin;C:\ST\STM32CubeIDE_1.16.1\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.make.win32_2.1.300.202402091052\tools\bin;" + $env:PATH
make -j4
```

File nạp tạo ra tại thư mục `build/`:
- `build/microros_H7.bin` (dùng để nạp qua STM32CubeProgrammer hoặc ST-Link Utility).
- `build/microros_H7.elf` (dùng để Debug bằng GDB / OpenOCD).

---

## 🔍 Hướng dẫn Đọc Log Gỡ lỗi (Debug UART7 @ PE8)

Khi kết nối chân **PE8** qua mạch USB-TTL và mở phần mềm Serial Monitor (PuTTY / MobaXterm @ **115200 bps**), bạn sẽ thấy các thông báo thời gian thực:

```text
========================================
[BOOT] STM32H723 Robot Controller Starting...
[BOOT] Debug Serial: UART7 @ 115200 bps (PE8 TX)
[BOOT] BMI088 IMU: OK
[BOOT] Launching FreeRTOS Kernel...
========================================

[ROS] StartDefaultTask running. Init USB custom transport...
[MOTOR] Motor_task started.
[MOTOR] Enabling FDCAN1 & FDCAN3 motors...
[MOTOR] FDCAN1 & FDCAN3 Filters Configured.
[INS] INS_task started (Mahony AHRS filter active).
[OBSERVE] Task started. Waiting for INS convergence...
[OBSERVE] INS Converged! Velocity Observer Kalman Filter Active.

[ROS] Waiting for micro-ROS Agent connection over USB CDC...
[ROS] micro-ROS Agent CONNECTED! Initializing ROS 2 entities...
[ROS] Node [stm32h7_node] Ready | Pub: [/motor_fb] | Sub: [/cmd_vel, /motor_enable]

[CMD_VEL] vx=0.30 m/s, wz=0.00 rad/s
[MOTOR_EN] State=1
```

Nếu hệ thống gặp lỗi phần cứng hoặc bộ nhớ, chip sẽ tự động in cảnh báo:
- `[CRASH] *** HardFault_Handler triggered! ***` -> Lỗi truy cập con trỏ NULL / sai vùng nhớ.
- `[FATAL] Stack Overflow in Task: [defaultTask]!` -> Tràn bộ nhớ Stack của task tương ứng.
- `[FATAL] FreeRTOS Heap Allocation Failed!` -> Hết bộ nhớ RAM Heap.
