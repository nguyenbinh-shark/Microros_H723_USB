# Porting, Configuration & Motor Commissioning

This guide explains how to adapt the firmware for different robot dimensions, gear ratios, motor models, and CAN ID configurations.

---

## 1. Central Robot Configuration (`robot_config.h`)

All core geometric parameters and hardware tuning gains are defined in [`User/Config/robot_config.h`](file:///p:/Prj_STM32/ros_h7_usb/User/Config/robot_config.h):

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
3. In [`User/APP/motor_task.c`](file:///p:/Prj_STM32/ros_h7_usb/User/APP/motor_task.c), update the motor ID macros:
   ```c
   #define MOTOR_ID_LEFT   1U
   #define MOTOR_ID_RIGHT  2U
   ```

---

## 3. Adapting to Other Actuators via `motor_drv` Layer

The firmware isolates motor-specific communication inside [`User/Devices/DRV_Motor/motor_drv.c`](file:///p:/Prj_STM32/ros_h7_usb/User/Devices/DRV_Motor/motor_drv.c). To support another motor protocol (such as DJI RoboMaster M3508/GM6020 or Unitree A1 actuators):

1. Implement your motor protocol driver under `User/Devices/DRV_Motor/`.
2. Map the generic callbacks in `motor_drv.c`:
   - `motor_enable()`
   - `motor_disable()`
   - `motor_send_velocity()`
   - `motor_parse_feedback()`
3. Ensure feedback positions and velocities are converted into standard SI units:
   - Position: Continuous unwrapped radians ($\text{rad}$).
   - Velocity: Hub angular velocity ($\text{rad/s}$).
   - Effort: Output torque ($\text{Nm}$).

