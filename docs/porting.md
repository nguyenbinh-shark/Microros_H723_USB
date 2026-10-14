# Porting, Configuration & Motor Commissioning

This guide explains how to adapt the firmware for different robot dimensions, gear ratios, motor models, and CAN ID configurations.

---

## 1. Central Robot Configuration (`robot_config.h`)

All core geometric parameters and hardware tuning gains are defined in [`User/Config/robot_config.h`](../User/Config/robot_config.h):

```c
/* ── Geometric Parameters ───────────────────────────────────────────── */
#define WHEEL_RADIUS        0.05f   /* Wheel radius in meters (5 cm) */
#define WHEEL_BASE          0.30f   /* Distance between wheels in meters (30 cm) */

/* ── Motor Directions (Mirrored Mounting) ───────────────────────────── */
#define MOTOR_LEFT_SIGN     ( 1.0f) /* +1.0f if positive velocity = FORWARD */
#define MOTOR_RIGHT_SIGN    (-1.0f) /* -1.0f if positive velocity = BACKWARD */

/* ── Transceiver & Tuning Gains ──────────────────────────────────────── */
#define CAN_XCVR_EN_LEVEL   GPIO_PIN_SET  /* CtrBoard-H7 uses active-HIGH enable */
#define TORQUE_P_GAIN       0.80f         /* Velocity tracking Kd gain in MIT mode */
#define CMD_VEL_TIMEOUT_MS  500U          /* Watchdog timeout in ms */
```

### Adapting to a Different Chassis Geometry
1. Measure your wheel outer radius (in meters) and update `WHEEL_RADIUS`.
2. Measure the center-to-center track width between the two drive wheels (in meters) and update `WHEEL_BASE`.
3. If your motor wheels rotate in reverse when receiving positive linear command, invert the corresponding `MOTOR_LEFT_SIGN` or `MOTOR_RIGHT_SIGN`.

---

## 2. Motor Commissioning & Dual-Bus Topology

### Why Both Motors Use CAN ID `1`
- In standard differential-drive robots with single-bus CAN, each actuator must have a unique ID (`1` and `2`).
- This robot uses **independent physical FDCAN peripherals**:
  - `FDCAN1` connects exclusively to the **Left Motor** (ID `1`).
  - `FDCAN3` connects exclusively to the **Right Motor** (ID `1`).
- **Advantages:**
  1. No motor ID reconfiguration needed when replacing a spare unit from factory stock.
  2. Maximum bus bandwidth (1 Mbps dedicated per motor).
  3. Bus fault on one motor does not physically crash the other channel.

### Setting RobStride / CyberGear Motor IDs
If you choose to operate both motors on a shared single CAN bus:
1. Connect each motor individually to a USB-to-CAN adapter.
2. Use the manufacturer host tool to configure Motor 1 to ID `0x01` and Motor 2 to ID `0x02`.
3. In [`User/APP/motor_task.c`](../User/APP/motor_task.c), update the motor ID macros:
   ```c
   #define MOTOR_ID_LEFT   1U
   #define MOTOR_ID_RIGHT  2U
   ```

---

## 3. Adapting to Other Actuators

Motor-specific CAN framing lives in [`User/Devices/DRV_Motor/`](../User/Devices/DRV_Motor/). The directory holds two drivers, and only one of them is live:

| File | Status | Protocol |
| :--- | :--- | :--- |
| [`robstride_drv.c`](../User/Devices/DRV_Motor/robstride_drv.c) | **Active** — what the robot runs on | RobStride, 29-bit extended (`rs_ext_*`) |
| [`cybergear_drv.c`](../User/Devices/DRV_Motor/cybergear_drv.c) | Reference only, no call sites | Xiaomi CyberGear |

`robstride_drv.c` carries **two** protocols. Section banners in the source mark them:

| Block | Protocol | Called? |
| :--- | :--- | :--- |
| `ACTIVE` (`rs_ext_*`) | 29-bit extended — RobStride / CyberGear family | Yes, this is what runs |
| `MIT 11-BIT MODE` (`rs_mit_cmd`, `rs_enable`, `rs_disable`, `rs_parse_feedback`) | Classic MIT-Cheetah: 11-bit standard ID, `P16\|V12\|Kp12\|Kd12\|T12` packing, `FF..FC` / `FF..FD` enable/disable | No — working code, just not wired up |

The MIT block is a complete second driver, not dead leftovers: use it for MIT-Cheetah / Damiao actuators, or RobStride units switched into MIT mode. Switching to it means changing the call sites in `motor_task.c` and `can_bsp.c`, not editing the block.

Read the banners before copying a constant: `RS_*` (MIT) and `RS_EXT_*` (29-bit) look alike but differ (V 33 vs 20 rad/s, KP 500 vs 5000, KD 5 vs 100). Note also that CyberGear has no 11-bit mode at all — its protocol is 29-bit only, so "CyberGear" and "MIT 11-bit" are never the same thing.

There is no generic dispatch layer: the firmware calls the RobStride driver directly, at exactly two seams.

1. Implement your protocol driver under `User/Devices/DRV_Motor/` and add it to `C_SOURCES` in the [`Makefile`](../Makefile). Use `cybergear_drv.c` as the shape of a second driver.
2. Swap the **command path** in [`User/APP/motor_task.c`](../User/APP/motor_task.c) — replace these three calls, made on `&hfdcan1` (left) and `&hfdcan3` (right):
   - `rs_ext_enable()`
   - `rs_ext_disable()`
   - `rs_ext_control_cmd()` — called once per motor at 100 Hz
3. Swap the **feedback path** in [`User/Bsp/can_bsp.c`](../User/Bsp/can_bsp.c) — `CAN_ParseMotorFeedback()` calls `rs_ext_parse_feedback()` to fill a per-motor slot. Keep the slot convention (`idx 0` = left/FDCAN1, `idx 1` = right/FDCAN3) and the rest of the stack (`CAN_MotorFeedback_Get_Idx()`, `CAN_GetMotorVel()`, odometry, diagnostics) needs no change.
3. Ensure feedback positions and velocities are converted into standard SI units:
   - Position: Continuous unwrapped radians ($\text{rad}$).
   - Velocity: Hub angular velocity ($\text{rad/s}$).
   - Effort: Output torque ($\text{Nm}$).

