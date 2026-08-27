# Troubleshooting & Diagnostic Guide

This guide provides diagnostic procedures and resolution steps for common hardware, CAN communication, and ROS 2 integration issues.

---

## 1. CAN Bus Diagnostics & UART7 `[CAN_PHY]` Logs

Connect a 3.3V USB-TTL serial adapter to pin `PE8` (UART7 TX) at **115200 bps, 8N1** to monitor live hardware diagnostics.

When CAN transmission issues occur, the firmware prints `[CAN_PHY]` diagnostics containing the **LEC (Last Error Code)** from the FDCAN Protocol Status Register:

| Logged Error Code | Hardware Root Cause | Recommended Action |
|---|---|---|
| `LEC=BIT0` | Transmitted dominant (0), read back recessive (1). The bus was not driven at all. | • Verify CAN transceivers have 5V/3.3V power.<br>• Verify transceiver enable pins (`PC13`, `PC14`, `PC15`) are driven **HIGH**.<br>• Check for broken `CAN_TX` / `CAN_RX` traces. |
| `LEC=ACK` | The transmitter drove the frame, but no receiving node replied with an ACK bit. | • Check motor power supply (24V).<br>• Verify motor CAN wiring (`CAN_H` to `CAN_H`, `CAN_L` to `CAN_L`, common GND).<br>• Ensure $120\text{ }\Omega$ termination resistors are present.<br>• Verify motor CAN ID is set to `1`. |
| `LEC=STUFF` or `LEC=FORM` | Bit stuffing violation or framing error. | • Baudrate mismatch between STM32 (1 Mbps) and motor actuator.<br>• Heavy electromagnetic noise or unterminated reflection. |
| `CAN1_BUSOFF` / `CAN3_BUSOFF` | Error counters exceeded limit; peripheral entered Bus-Off. | • Power cycle motor bus and check cable shielding. |

---

## 2. ROS 2 Communication Issues

### Issue A: `ros2 topic echo /motor_fb` produces no output
- **Root Cause:** QoS profile mismatch. `motor_fb` is published with **Best Effort** reliability. By default, `ros2 topic echo` expects Reliable QoS.
- **Resolution:**
  ```bash
  ros2 topic echo /motor_fb --qos-reliability best_effort
  ```

### Issue B: Node is visible and `/cmd_vel` is publishing, but robot wheels do not rotate
- **Root Cause:** By default, motors boot into a safe **DISABLED** state (`/motor_enable` is false).
- **Resolution:**
  - If using `teleop_key.py`: Press the **`e`** key to enable motors.
  - Via CLI:
    ```bash
    ros2 topic pub --once /motor_enable std_msgs/msg/Bool "{data: true}"
    ```

### Issue C: `/dev/stm32_robot` not found
- **Root Cause:** UDEV rule not installed or USB cable not connected.
- **Resolution:**
  1. Check system logs: `dmesg | tail -n 20` (look for `STMicroelectronics STM32 STLink` or `Virtual COM Port`).
  2. Install UDEV rules:
     ```bash
     cd deploy/
     sudo ./install_host.sh
     ```

### Issue D: `Permission denied: '/dev/ttyACM0'`
- **Root Cause:** Current Linux user lacks serial device permissions.
- **Resolution:**
  ```bash
  sudo usermod -aG dialout $USER
  # Log out and log back in, or run:
  newgrp dialout
  ```

---

## 3. Microcontroller Debugging & Crash Analysis

- **Stack Overflow:** If a FreeRTOS task exceeds its allocated stack, `vApplicationStackOverflowHook` catches it and prints the faulting task name over UART7 before halting.
- **HardFault Handler:** In case of memory access violations or unaligned access, the default `HardFault_Handler` loops and can be inspected via OpenOCD / GDB.
- **Mailing / Logging Bug Reports:** When reporting issues, please include:
  1. Full UART7 boot banner and runtime logs.
  2. Output of `ros2 topic echo /motor_status`.

