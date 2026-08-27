# Firmware Architecture & Development Guide

This guide describes the FreeRTOS multitasking architecture, source tree layout, and debugging procedures for the STM32H723 micro-ROS robot firmware.

---

## 1. FreeRTOS Task Architecture

The firmware runs five concurrent FreeRTOS tasks under CMSIS-RTOS v2 API:

| Task Name | Function Entry | Rate / Period | Priority | Stack Size | Primary Responsibility |
|---|---|:---:|---|:---:|---|
| `insTask` | `INS_Task_Entry` | **1 kHz (1 ms)** | `osPriorityRealtime` | 4 KB | Reads BMI088 IMU via SPI2; runs Mahony AHRS filter to compute attitude quaternion & Euler angles. |
| `observeTask` | `Observe_Task_Entry` | **333 Hz (3 ms)** | `osPriorityHigh` | 4 KB | Executes Kalman Velocity Observer to fuse wheel odometry with IMU linear acceleration. |
| `motorTask` | `Motor_Task_Entry` | **100 Hz (10 ms)** | `osPriorityAboveNormal` | 4 KB | Runs differential kinematics, formats FDCAN operation control frames, dispatches commands to actuators. |
| `defaultTask` | `StartDefaultTask` | **50 Hz (20 ms)** | `osPriorityNormal` | **12 KB** | Manages micro-ROS session lifecycle, spins executor, publishes `/motor_fb` (50 Hz) and `/motor_status` (5 Hz). |
| `lcdTask` | `LCD_Task_Entry` | **10 Hz (100 ms)** | `osPriorityBelowNormal` | 4 KB | Renders telemetry, ping RTT, topic rates, and bus diagnostics onto ST7789 display. |

```text
  Real-Time Interrupts & High-Priority Tasks
  ===========================================
  [1000 Hz] insTask (Realtime)      --> Read SPI2 IMU & Mahony EKF
  [ 333 Hz] observeTask (High)      --> Fuse IMU + Wheel Odometry
  [ 100 Hz] motorTask (AboveNormal) --> CAN Dispatch & Safety Watchdog
  [  50 Hz] defaultTask (Normal)    --> micro-ROS Executor & Pub/Sub
  [  10 Hz] lcdTask (BelowNormal)   --> ST7789 UI Refresh
```

---

## 2. Source Tree Layout

```text
Microros_H723_USB/
├── Core/
│   ├── Inc/               # CubeMX generated peripheral headers & main.h
│   └── Src/
│       ├── freertos.c     # FreeRTOS setup & micro-ROS integration (USER CODE blocks)
│       └── main.c         # Clock config (192 MHz) & hardware initialization
├── Drivers/
│   ├── CMSIS/             # ARM CMSIS Core & DSP library
│   └── STM32H7xx_HAL_Driver/ # ST HAL driver library
├── Middlewares/
│   ├── ST/STM32_USB_Device_Library/ # ST USB CDC stack
│   └── Third_Party/FreeRTOS/       # FreeRTOS kernel source
├── micro_ros_stm32cubemx_utils/     # micro-ROS platform glue & USB CDC transport
├── User/
│   ├── Algorithm/         # Control algorithms: PID, Kinematics, Mahony AHRS, Kalman
│   ├── APP/               # High-level FreeRTOS tasks (motor_task, INS_task, observe_task, lcd_task)
│   ├── Bsp/               # Board support package (can_bsp, bsp_dwt, bsp_PWM)
│   ├── Config/            # Global geometry and robot definitions (robot_config.h)
│   ├── Devices/           # Hardware drivers (DRV_Motor, BMI088, LCD)
│   └── Lib/               # Math utilities and filters
├── deploy/                # UDEV rules and systemd deployment scripts for host PC
├── docs/                  # System documentation and references
├── teleop/                # Python keyboard teleoperation script
└── tools/                 # Setup scripts (fetch_libmicroros.sh, build_libmicroros.sh)
```

---

## 3. micro-ROS Integration Details

All micro-ROS initialization and entity management reside in `Core/Src/freertos.c`:

1. **State Machine (`g_agent_state`):**
   - `WAITING_AGENT`: Pings micro-ROS agent via `rmw_uros_ping_agent()`.
   - `AGENT_AVAILABLE`: Creates node, publishers, subscribers, and executor.
   - `AGENT_CONNECTED`: Spins executor and publishes feedback. Also executes periodic ping (100 ms) and time synchronization (every 60 s).
   - `AGENT_DISCONNECTED`: Safely stops motors (`robot_cmd_set(0, 0, 0)`), tears down DDS entities, and returns to `WAITING_AGENT`.
2. **Dynamic Memory Allocation:**
   - Static memory pool managed by `custom_memory_manager.c` to prevent heap fragmentation in FreeRTOS.

---

## 4. Debugging with VS Code & GDB

To debug using VS Code and ST-LINK / OpenOCD, use the following `.vscode/launch.json` configuration:

```json
{
  "version": "0.2.0",
  "configurations": [
    {
      "name": "Cortex-Debug (OpenOCD)",
      "type": "cortex-debug",
      "request": "launch",
      "servertype": "openocd",
      "cwd": "${workspaceFolder}",
      "executable": "${workspaceFolder}/build/microros_H7.elf",
      "configFiles": [
        "interface/stlink.cfg",
        "target/stm32h7x.cfg"
      ],
      "svdFile": "STM32H723.svd",
      "runToEntryPoint": "main"
    }
  ]
}
```

### Serial Printf Output
- `printf()` is redirected to **UART7** (pin `PE8`) at **115200 baud, 8N1** in `Core/Src/usart.c` / `syscalls.c`.

