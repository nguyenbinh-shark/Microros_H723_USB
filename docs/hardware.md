# Hardware Architecture & Wiring Guide

This document describes the hardware specifications, Bill of Materials (BOM), wiring diagrams, and pin assignments for the STM32H723 differential-drive robot controller.

---

## 1. System Overview & Bill of Materials (BOM)

| Component | Model / Specification | Description |
|---|---|---|
| **MCU Board** | CtrBoard-H7 (STM32H723VGT6) | ARM Cortex-M7 @ 192 MHz, 1024 KB Flash, 560 KB RAM |
| **Motors** | 2× RobStride Actuators (or CyberGear) | BLDC planetary geared actuators with integrated FOC drivers |
| **IMU** | Bosch BMI088 (onboard / SPI2) | 6-DOF high-precision IMU (Accel ±6g, Gyro ±2000°/s) |
| **Display** | ST7789V IPS (240×280 px / SPI1) | Color SPI display for real-time status and diagnostics |
| **Host PC** | Raspberry Pi 4/5, Jetson, or PC | Runs ROS 2 Jazzy + micro-ROS Agent |
| **USB Link** | USB-C to USB-A/C Data Cable | High-Speed USB CDC Virtual COM Port (12 Mbps PHY) |
| **Debug Port** | USB-to-UART Adapter (3.3V) | Connected to UART7 (PE8 TX) for diagnostic printout |
| **Power Supply**| 24V LiPo / DC Power Supply | Dedicated motor supply + regulated 5V step-down for MCU |

---

## 2. Electrical Wiring & CAN Bus Topology

```text
+-------------------+                          +-------------------+
|  Left RobStride   |                          |  Right RobStride  |
|  CAN ID: 1        |                          |  CAN ID: 1        |
+---------+---------+                          +---------+---------+
          | CANH / CANL (120Ω Term)                      | CANH / CANL (120Ω Term)
          |                                              |
    [ FDCAN1 Port ]                                [ FDCAN3 Port ]
          |                                              |
+---------+----------------------------------------------+---------+
|                                                                  |
|               CtrBoard-H7 (STM32H723VGT6 @ 192 MHz)              |
|                                                                  |
+-------------------+----------------------+-----------------------+
                    |                      |
             [ USB CDC ACM ]         [ UART7 TX ]
                    |                      |
            +-------+-------+      +-------+-------+
            |  Host Computer |      | USB-TTL Debug |
            | (ROS 2 Jazzy) |      | (115200 8N1)  |
            +---------------+      +---------------+
```

### Critical Wiring Rules

1. **Independent Dual CAN Buses:**
   - **Left Motor:** Connected to `FDCAN1` (Pins `PD0`/`PD1`).
   - **Right Motor:** Connected to `FDCAN3` (Pins `PD12`/`PD13`).
   - Both motors use CAN ID `1` on their respective buses.
2. **Termination Resistors (120 Ω):**
   - Each CAN bus must have $120\text{ }\Omega$ termination enabled between `CAN_H` and `CAN_L` at both ends of the bus.
3. **Common Ground:**
   - Always connect logic ground (GND) of the CtrBoard-H7 to the motor power supply ground (GND). Never rely solely on CAN differential lines without ground reference.
4. **CAN Transceiver Enable:**
   - On CtrBoard-H7, the onboard CAN transceivers have **active-HIGH** enable lines controlled by GPIOs `PC13`, `PC14`, and `PC15`. Firmware automatically drives these pins HIGH on initialization.

---

## 3. Microcontroller Pinout Table

### CAN Interfaces

| Function | Pin | Direction | Notes |
|---|---|---|---|
| `FDCAN1_RX` | `PD0` | Input | Left Motor CAN RX |
| `FDCAN1_TX` | `PD1` | Output | Left Motor CAN TX |
| `FDCAN3_RX` | `PD12` | Input | Right Motor CAN RX |
| `FDCAN3_TX` | `PD13` | Output | Right Motor CAN TX |
| `CAN1_EN` | `PC13` | Output | Active-HIGH transceiver enable |
| `CAN2_EN` | `PC14` | Output | Active-HIGH transceiver enable |
| `CAN3_EN` | `PC15` | Output | Active-HIGH transceiver enable |

### USB & Diagnostics

| Function | Pin | Direction | Notes |
|---|---|---|---|
| `USB_OTG_HS_DM` | `PA11` | Bi-directional | USB CDC Data Minus (Internal FS PHY) |
| `USB_OTG_HS_DP` | `PA12` | Bi-directional | USB CDC Data Plus (Internal FS PHY) |
| `UART7_TX` | `PE8` | Output | Diagnostic logging @ 115200 bps, 8N1 |
| `UART7_RX` | `PE7` | Input | Optional serial receive |

### Onboard BMI088 IMU (SPI2)

| Function | Pin | Description |
|---|---|---|
| `SPI2_SCK` | `PB13` | Serial Clock |
| `SPI2_MISO`| `PB14` | Master In / Slave Out |
| `SPI2_MOSI`| `PB15` | Master Out / Slave In |
| `ACC_CS` | `PC0` | Accelerometer Chip Select (Active-LOW) |
| `GYRO_CS`| `PC3` | Gyroscope Chip Select (Active-LOW) |
| `ACC_INT` | `PE10`| Accelerometer Data Ready Interrupt |
| `GYRO_INT`| `PE12`| Gyroscope Data Ready Interrupt |

### Onboard ST7789V Display (SPI1)

| Function | Pin | Description |
|---|---|---|
| `SPI1_SCK` | `PB3` | LCD Clock |
| `SPI1_MOSI`| `PB5` | LCD Data In |
| `LCD_CS` | `PE15` | LCD Chip Select |
| `LCD_DC` | `PD10` | LCD Data/Command Select |
| `LCD_RES` | `PB11` | LCD Hardware Reset |
| `LCD_BLK` | `PB10` | LCD Backlight Control |
