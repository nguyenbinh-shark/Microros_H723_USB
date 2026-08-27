# RobStride / CyberGear CAN Protocol Summary

This document summarizes the CAN communication protocol implemented in `User/Devices/DRV_Motor/robstride_drv.c` for RobStride and CyberGear-compatible brushless actuator modules.

---

## 1. CAN Frame Architecture

The driver uses **CAN 2.0B 29-bit Extended Identifiers** structured into three functional bitfields:

```text
 28      24 23                      8 7            0
+----------+-------------------------+--------------+
| Type (5) |  Data / Source ID (16)  | Target ID (8)|
+----------+-------------------------+--------------+
```

- **Bits [28:24] — Communication Type (5 bits):** Identifies the command or message category.
- **Bits [23:8] — Data / Source Info (16 bits):**
  - For command frames: Parameter index or 16-bit packed data (e.g. Feedforward Torque).
  - For feedback frames: Bits [15:8] = Motor CAN ID, Bits [7:0] = Master CAN ID.
- **Bits [7:0] — Target Motor CAN ID (8 bits):** Target motor ID (`0x01`–`0x7F`, broadcast `0x00`).

---

## 2. Command Types Used in Firmware

| Type | Name | Purpose | CAN ID Payload | 8-Byte Data Payload |
|:---:|---|---|---|---|
| `0x01` | **Operation Control** | Real-time motion command (MIT mode) | Bit[23:8] = Target Torque ($T_{ff}$) | Pos (2B) + Vel (2B) + $K_p$ (2B) + $K_d$ (2B) |
| `0x02` | **Feedback Frame** | Motor status feedback | Bit[15:8] = Motor ID, Bit[7:0] = Master ID | Pos (2B) + Vel (2B) + Torque (2B) + Temp (2B) |
| `0x03` | **Motor Enable** | Transition motor into RUN mode | Master ID in Data2 field | `[0x00..0x00]` (8 bytes zero) |
| `0x04` | **Motor Disable** | Disable motor / clear faults | Master ID in Data2 field | Byte 0: `0` (stop) or `1` (clear fault) |
| `0x11` | **Read Parameter** | Query single RAM/EEPROM index | Master ID in Data2 | Byte 0..1: Index (little-endian) |
| `0x12` | **Write Parameter** | Modify control mode / limits | Master ID in Data2 | Byte 0..1: Index, Byte 4..7: Value (float/u32) |

---

## 3. Real-Time Operation Control (Type `0x01`)

The firmware drives the motors using Operation Control Mode (MIT impedance control):

$$\tau = \tau_{ff} + K_p (p_{des} - p_{act}) + K_d (v_{des} - v_{act})$$

For closed-loop velocity control as used in differential drive:
- $p_{des} = 0$, $K_p = 0$
- $v_{des} =$ Target angular velocity (rad/s)
- $K_d = \text{TORQUE\_P\_GAIN}$ (acting as velocity P gain)
- $\tau_{ff} = 0$

### Scaling and Quantization Limits

| Field | Range | Integer Bits | Resolution |
|---|---|:---:|---|
| Position ($p$) | $[-12.57, +12.57]\text{ rad}$ | 16-bit uint | $\approx 0.00038\text{ rad}$ |
| Velocity ($v$) | $[-20.0, +20.0]\text{ rad/s}$ | 16-bit uint | $\approx 0.00061\text{ rad/s}$ |
| $K_p$ | $[0.0, 5000.0]$ | 16-bit uint | $\approx 0.076$ |
| $K_d$ | $[0.0, 100.0]$ | 16-bit uint | $\approx 0.0015$ |
| Torque ($\tau$) | $[-60.0, +60.0]\text{ Nm}$ | 16-bit uint | $\approx 0.0018\text{ Nm}$ |

---

## 4. Feedback Frame (Type `0x02`)

The actuator transmits feedback frames periodically (or in response to control frames):

- **Data Bytes 0–1:** Unscaled 16-bit raw angle ($p_{raw} \in [-12.57, 12.57]\text{ rad}$).
- **Data Bytes 2–3:** Unscaled 16-bit velocity ($v_{raw} \in [-20.0, 20.0]\text{ rad/s}$).
- **Data Bytes 4–5:** Unscaled 16-bit torque ($\tau_{raw} \in [-60.0, 60.0]\text{ Nm}$).
- **Data Bytes 6–7:** Temperature ($T = \text{raw} / 10.0\text{ }^\circ\text{C}$).
- **CAN ID Bits [23:16]:** Status bitfield (`0` = Reset, `1` = Calibrating, `2` = Motor Running, plus fault flags).

Multi-turn angle unwrapping is handled in software (`rs_ext_parse_feedback`) by tracking $[-\pi, +\pi]$ boundary crossings.

---

## 5. Master ID & Multi-Bus Topology Convention

- By convention, the host microcontroller sets `MASTER_ID = 0xFD`.
- In this robot architecture, each motor is configured with CAN ID `0x01`, separated on independent hardware buses:
  - **Left Motor:** CAN ID `1`, attached to `FDCAN1`.
  - **Right Motor:** CAN ID `1`, attached to `FDCAN3`.
- This dual-bus design isolates communication traffic, allows identical spare replacement without re-flashing motor EEPROM, and enables full 1 Mbps bandwidth per channel.

---

## 6. Official References

For complete hardware datasheets and manufacturer-specific parameter tables:
- RobStride Official Documentation & Software: [RobStride Manufacturer Site](https://www.robstride.com)
- Xiaomi CyberGear Micro Motor User Manual: Public OEM Documentation & Protocols.
