# Notices and Open Source Attribution

This project (`ros_h7_usb`) is licensed under the **MIT License** for all custom firmware code, integration logic, deployment scripts, and documentation written by the repository maintainer (Tran Nguyen Binh).

This repository contains and builds upon third-party open-source components and vendor libraries. Their respective licenses, copyrights, and attributions are detailed below.

---

## 1. Primary Components & Libraries

### STM32H7 HAL & LL Drivers
- **Path:** `Drivers/STM32H7xx_HAL_Driver/`, `Drivers/CMSIS/Device/ST/STM32H7xx/`
- **Copyright:** © STMicroelectronics
- **License:** BSD-3-Clause
- **Notice:** Licensed under STMicroelectronics BSD-3-Clause license terms.

### ARM CMSIS & CMSIS-DSP
- **Path:** `Drivers/CMSIS/Include/`, `Drivers/CMSIS/DSP/Include/`, `Drivers/CMSIS/DSP/Lib/GCC/libarm_cortexM7lfdp_math.a`
- **Copyright:** © ARM Limited
- **License:** Apache-2.0

### FreeRTOS Real-Time Operating System
- **Path:** `Middlewares/Third_Party/FreeRTOS/`
- **Copyright:** © Amazon.com, Inc. or its affiliates
- **License:** MIT License

### STMicroelectronics USB Device Library
- **Path:** `Middlewares/ST/STM32_USB_Device_Library/`
- **Copyright:** © STMicroelectronics
- **License:** ST Ultimate Liberty License (SLA0044)

### micro-ROS STM32CubeMX Utilities & Prebuilt Static Library
- **Path:** `micro_ros_stm32cubemx_utils/`, `tools/fetch_libmicroros.sh`
- **Copyright:** © eProsima / Open Source Robotics Foundation (OSRF)
- **License:** Apache-2.0
- **Reference:** See `micro_ros_stm32cubemx_utils/3rd-party-licenses.txt` for the full dependency attribution list.

---

## 2. Algorithms and Embedded Modules (`User/`)

### DJI PID Controller
- **Path:** `User/Algorithm/PID/pid.c`, `User/Algorithm/PID/pid.h`
- **Copyright:** © 2016 DJI
- **Attribution:** Original header comments and copyright notices are preserved intact.

### RoboMaster Community & Wang Hongxi Modules
- **Path:**
  - `User/Bsp/bsp_dwt.*`
  - `User/Bsp/bsp_PWM.*`
  - `User/Lib/user_lib.*`
  - `User/Algorithm/kalman/kalman_filter.*`
  - `User/Algorithm/EKF/QuaternionEKF.*`
  - `User/Algorithm/mahony/mahony_filter.*`
- **Author/Provenance:** Wang Hongxi (RoboMaster community).
- **Notice:** Original file headers and authorship metadata are retained intact. These files are subject to their respective original authors' rights and are not granted under the project MIT license.

### BMI088 IMU Driver
- **Path:** `User/Devices/BMI088/BMI088driver.*`, `User/Devices/BMI088/BMI088Middleware.*`
- **Notice:** Embedded driver for Bosch Sensortec BMI088 6-DOF IMU. Original attribution retained.

### ST7789 LCD Driver & Font Table
- **Path:** `User/Devices/LCD/lcd.*`, `User/Devices/LCD/lcdfont.h`
- **Notice:** Embedded display rendering routines for ST7789 SPI display controllers. Original attribution retained.

