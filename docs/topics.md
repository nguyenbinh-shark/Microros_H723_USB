# ROS 2 Topics & Interface Specification

This document details the ROS 2 interface contract implemented by the `stm32h7_node` firmware on STM32H723.

---

## 1. Node Identity

- **Node Name:** `/stm32h7_node`
- **Namespace:** `""` (root namespace)
- **Transport:** USB CDC ACM (Virtual Serial) -> `micro_ros_agent`

---

## 2. Topic Summary Table

| Topic Name | Direction | Message Type | QoS Reliability | Rate | Purpose |
|---|:---:|---|---|:---:|---|
| `/cmd_vel` | Subscriber | `geometry_msgs/msg/Twist` | **Best Effort** | Up to 50 Hz | Robot velocity command, drives motors in **Velocity mode** |
| `/cmd_mit` | Subscriber | `std_msgs/msg/Float64MultiArray` | **Best Effort** | Up to 500 Hz | Per-wheel MIT setpoint, drives motors in **MIT mode** |
| `/cmd_torque` | Subscriber | `std_msgs/msg/Float64MultiArray` | **Best Effort** | Up to 500 Hz | Per-wheel torque, switches motors to **MIT mode** |
| `/motor_enable` | Subscriber | `std_msgs/msg/Bool` | **Reliable** | On-demand | Enable (`true`) or Disable (`false`) motor outputs |
| `/motor_fb` | Publisher | `sensor_msgs/msg/JointState` | **Best Effort** | **50 Hz** | Position, velocity, and torque feedback from wheels |
| `/motor_status` | Publisher | `std_msgs/msg/UInt8` | **Reliable** | **5 Hz** | System health, CAN status, timeout and control-mode bitmask |
| `/imu/data` | Publisher | `sensor_msgs/msg/Imu` | **Best Effort** | **50 Hz target** | Calibrated gyro, gravity-inclusive acceleration and relative orientation |
| `/imu/debug` | Publisher | `std_msgs/msg/Float64MultiArray` | **Best Effort** | **5 Hz target** | IMU initialization, raw readings and calibration progress |

---

## 3. Command Topics and Control Modes

Each command topic maps to exactly one RobStride actuator `run_mode`:

| Topic | Actuator mode | `run_mode` | What the host controls |
|---|---|:---:|---|
| `/cmd_vel` | Velocity | `2` | Robot $v_x$, $\omega_z$. The motor's own PI loop holds wheel speed. |
| `/cmd_mit` | Operation Control (MIT) | `0` | Raw $p$, $v$, $K_p$, $K_d$, $\tau$ per wheel. |
| `/cmd_torque` | Operation Control (MIT) | `0` | $\tau$ per wheel only; $p$, $v$, $K_p$, $K_d$ are sent as 0. |

**The topic that published last selects the mode.** When a message arrives on a topic whose mode differs from the active one, `motorTask` sends stop → write `run_mode` → enable to both motors. The wheels are unpowered for those few CAN frames, so a host should drive one command topic at a time and never interleave them.

- **Boot mode:** Velocity, until the first `/cmd_mit` or `/cmd_torque` message arrives. Those two share MIT mode, so switching between them costs no mode change.
- **RC override:** with the SBUS transmitter in `MANUAL_RC`, the sticks produce a $(v, \omega)$ pair, so the motors run in Velocity mode regardless of which ROS topic spoke last.
- **RC switches** (`User/Bsp/bsp_sbus.h`), highest priority first:
  - **F (CH10), push button:** a press latches the E-stop. Switching E off and then on, with F released, resets it.
  - **No RC link** (receiver failsafe or no frame for 300 ms): the wheels stop.
  - **E (CH9):** enables or disables the motors, like `/motor_enable`.
  - **A (CH5):** down selects AUTO (the ROS command topics drive the robot); any other position selects MANUAL (the sticks drive it).
  - The LCD shows `RC:ESTOP`, `RC:LOST`, `RC:DISARM`, `RC:MANUAL` or `RC:AUTO`.
- **Safety Timeout:** If the active command topic is silent for `CMD_VEL_TIMEOUT_MS` (500 ms), the firmware holds the wheels in the current mode and raises `MSTAT_CMD_TIMEOUT`:
  - Velocity mode: `SPD_REF = 0 rad/s`, actively held by the motor's PI loop.
  - MIT mode: $K_p = 0$, $v = 0$, $K_d = 0.8$, $\tau = 0$ — damping only, the wheel can still be pushed.

### 3.1 `/cmd_vel` (Velocity Command → Velocity mode)

- **Type:** `geometry_msgs/msg/Twist`
- **Subscribed Fields:**
  - `linear.x` (float64, interpreted as float32): Forward linear velocity in **$\text{m/s}$**.
  - `angular.z` (float64, interpreted as float32): Yaw rotational velocity in **$\text{rad/s}$**.
- **Differential Kinematics:**
  $$\omega_L = \frac{v_x - \frac{W}{2}\omega_z}{R}, \quad \omega_R = \frac{v_x + \frac{W}{2}\omega_z}{R}$$
  where $R = \text{WHEEL\_RADIUS} = 0.060\text{ m}$ and $W = \text{WHEEL\_BASE} = 0.345\text{ m}$, matching the current Pi bridge/URDF profile. `MOTOR_LEFT_SIGN` / `MOTOR_RIGHT_SIGN` are applied, then the result is written to the motor's `SPD_REF` parameter.
- **Saturation:** if either wheel would exceed $20\text{ rad/s}$ ($1.2491\text{ m/s}$), **both** wheel speeds are scaled by the same factor so the faster one lands on the limit. The curvature $\omega_z / v_x$ is preserved and the robot stays on the planned path; it only moves slower than commanded. Keep $|v_x| + \frac{W}{2}|\omega_z| \le 1.2491\text{ m/s}$ in the Nav2 limits to avoid scaling entirely.
- **Validation:** a message with NaN/Inf in `linear.x` or `angular.z` is dropped and logged on UART7.
- **Acceleration:** ramped inside the motor at `VEL_MODE_ACC_RAD` = $20\text{ rad/s}^2$ ($1.2491\text{ m/s}^2$ at the wheel).

```bash
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.0}}"
```

### 3.2 `/cmd_mit` (MIT Command → MIT mode)

- **Type:** `std_msgs/msg/Float64MultiArray` (same message as ros2_control's `forward_command_controller`)
- **`data` (exactly 10 values):**

| Index | Field | Unit | Range |
|:---:|---|---|---|
| `0` | Left position $p$ | rad | $[-12.57, 12.57]$ |
| `1` | Left velocity $v$ | rad/s | $[-20, 20]$ |
| `2` | Left $K_p$ | Nm/rad | $[0, 5000]$ |
| `3` | Left $K_d$ | Nm/(rad/s) | $[0, 100]$ |
| `4` | Left feedforward torque $\tau$ | Nm | $[-60, 60]$ |
| `5..9` | Right wheel, same order | | |

- **Control law (inside the actuator):** $\tau_{out} = \tau + K_p (p - p_{act}) + K_d (v - v_{act})$
- **Frame:** wheel frame, the same frame `/motor_fb` reports in: positive $p$, $v$, $\tau$ drive **both** wheels forward. The firmware multiplies $p$, $v$ and $\tau$ by `MOTOR_*_SIGN` before sending them to the actuator; $K_p$ and $K_d$ are passed through unchanged.
- **Validation:** a message with `data.size != 10` or any NaN/Inf value is dropped and logged on UART7. Values outside the ranges above are clamped.
- **`layout`:** ignored; up to 2 dimensions with labels ≤ 16 chars are accepted so publishers that fill it in still get through.

> [!WARNING]
> With $K_p > 0$, the motor pulls towards $p$ immediately. Read the current position from `/motor_fb` and start from there, or the wheel jumps.

```bash
# Pure velocity damping on both wheels: v = 2 rad/s, Kd = 1.5
ros2 topic pub -r 100 /cmd_mit std_msgs/msg/Float64MultiArray \
  "{data: [0.0, 2.0, 0.0, 1.5, 0.0,  0.0, 2.0, 0.0, 1.5, 0.0]}"
```

### 3.2.1 `/cmd_torque` (Torque Command → MIT mode)

- **Type:** `std_msgs/msg/Float64MultiArray` (same message as ros2_control's `effort_controllers/JointGroupEffortController`)
- **`data` (exactly 2 values):** `[tau_left, tau_right]` in **Nm**, wheel frame: positive drives the robot **forward** on both sides, the same convention as `/motor_fb`.
- **Mode:** the first message switches the motors to MIT mode on its own. No other topic is needed.
- **What reaches the actuator:** an MIT setpoint with $p = v = K_p = K_d = 0$, so $\tau_{out} = \tau$.
- **Limit:** each value is clamped to $\pm$`CMD_TORQUE_MAX_NM` (`robot_config.h`, default **10 Nm**), then multiplied by `MOTOR_*_SIGN`.
- **Validation:** `data.size != 2` or a NaN/Inf value drops the message and logs on UART7.
- **Timeout:** the same 500 ms watchdog as the other command topics. On timeout, the motors fall back to MIT damping ($K_d = 0.8$, $\tau = 0$).

> [!WARNING]
> Pure torque has no speed loop and no damping. A constant torque keeps accelerating a free wheel until the actuator's own limit. The host must close the loop, for example by reading `/motor_fb` velocity, and stream at a steady rate.

```bash
# 0.5 Nm forward on both wheels, lift the robot off the ground first
ros2 topic pub -r 100 /cmd_torque std_msgs/msg/Float64MultiArray "{data: [0.5, 0.5]}"
```

### 3.3 `/motor_enable` (Motor State Control)

- **Type:** `std_msgs/msg/Bool`
- **Data:**
  - `true`: Writes the active `run_mode` and its parameters, then sends extended CAN enable frames to Left and Right RobStride motors.
  - `false`: Sends CAN stop frames to both motors and cuts torque.

### 3.4 `/motor_fb` (Wheel Odometry & Joint Feedback)

- **Type:** `sensor_msgs/msg/JointState`
- **Publish Rate:** 50 Hz target (every 20 ms in `defaultTask`), after agent time synchronization succeeds. USB and agent servicing can reduce the measured rate.
- **Header:**
  - `header.stamp`: Synchronized ROS 2 epoch timestamp (via micro-ROS time agent sync).
  - `header.frame_id`: `"base_link"`
- **Arrays (Size 2), wheel frame:** `MOTOR_*_SIGN` is already applied, so positive position, velocity and effort mean the wheel drives the robot **forward** on both sides. Host odometry can use the values directly: $v = R\,(\omega_R + \omega_L)/2$, $\omega_z = R\,(\omega_R - \omega_L)/W$.
  - `name`: `["left_wheel", "right_wheel"]`
  - `position`: `[pos_left_rad, pos_right_rad]` (continuous unwrapped radians)
  - `velocity`: `[vel_left_rad_s, vel_right_rad_s]` (rad/s)
  - `effort`: `[torq_left_nm, torq_right_nm]` (measured torque in Nm)

### 3.5 `/motor_status` (Status Bitmask)

- **Type:** `std_msgs/msg/UInt8`
- **Publish Rate:** 5 Hz (every 200 ms)
- **Bit Allocation:**

| Bit | Hex Mask | Flag Name | Description |
|:---:|:---:|---|---|
| `0` | `0x01` | `MSTAT_LEFT_FRESH` | Left motor CAN feedback received within timeout |
| `1` | `0x02` | `MSTAT_RIGHT_FRESH`| Right motor CAN feedback received within timeout |
| `2` | `0x04` | `MSTAT_CAN1_BUSOFF`| FDCAN1 is in Bus-Off state (error condition) |
| `3` | `0x08` | `MSTAT_CAN3_BUSOFF`| FDCAN3 is in Bus-Off state (error condition) |
| `4` | `0x10` | `MSTAT_MOTORS_ENABLED` | Motors are currently enabled |
| `5` | `0x20` | `MSTAT_CMD_TIMEOUT`| Active command topic silent for > 500 ms |
| `6` | `0x40` | `MSTAT_MODE_MIT` | `0` = Velocity mode (`/cmd_vel`), `1` = MIT mode (`/cmd_mit` or `/cmd_torque`) |
| `7` | `0x80` | *Reserved* | Reserved for future expansion |

---

### 3.6 `/imu/data` (BMI088 IMU)

- **Type:** `sensor_msgs/msg/Imu`; Best Effort, 50 Hz target.
- **Frame:** `imu_link`, using the physical BMI088 sensor axes. The host must provide the measured mounting transform between `base_link` and `imu_link`. The legacy application's swapped Roll/Pitch labels are not used to construct this message.
- **Orientation:** normalized quaternion, `w=INS.q[0]`, `x=q[1]`, `y=q[2]`, `z=q[3]`. Tilt is seeded from gravity; yaw is relative to the completed stationary startup calibration. BMI088 has no magnetometer or absolute heading reference.
- **Angular velocity:** rad/s, corrected by configured driver offsets plus the residual gyro bias learned at startup.
- **Linear acceleration:** m/s², **including gravity**. A stationary board with its sensor Z axis upward should report approximately `z=+9.81`; this is not `MotionAccel_b`.
- **Timestamp:** synchronized agent epoch time adjusted for the sample's age, rather than the publication time. Duplicate samples, non-increasing IMU timestamps and samples older than 50 ms are skipped.
- **Covariance:** diagonal, set in `robot_config.h`. Gyro variance includes margin above stationary measurements on this board (standard deviation 0.0105–0.0124 rad/s over 30 s); the other values remain initial estimates. Re-measure with the final mounting, temperature and motors running, then update the macros if needed.

| Field | Diagonal variance | Macro |
|---|---|---|
| `angular_velocity_covariance` | $2.5\times10^{-4}\ (\text{rad/s})^2$ ($\sigma \approx 0.0158$ rad/s) | `IMU_GYRO_VARIANCE` |
| `linear_acceleration_covariance` | $1.0\times10^{-2}\ (\text{m/s}^2)^2$ ($\sigma \approx 0.1$ m/s², allows for vibration) | `IMU_ACCEL_VARIANCE` |
| `orientation_covariance` roll, pitch | $1.0\times10^{-3}\ \text{rad}^2$ ($\sigma \approx 1.8°$) | `IMU_ORIENT_RP_VARIANCE` |
| `orientation_covariance` yaw | $1.0\times10^{6}\ \text{rad}^2$ (effectively unknown) | `IMU_ORIENT_YAW_VARIANCE` |

  The yaw variance is deliberately huge: yaw is relative to startup and drifts without bound. For `robot_localization`, fuse wheel velocity and gyro Z ($\omega_z$) only, and leave quaternion yaw out of `imu0_config`.
- **Publication gate:** requires a valid, calibrated, fresh sample and synchronized time. Invalid SPI reads, non-finite data and invalid AHRS updates suppress IMU publication. After synchronization fails, timestamped sensor publication pauses while synchronization is retried every second.

At startup, keep the board physically still for **at least 4 seconds**: 1 second warmup followed by 3 continuously stable seconds. Motion or excessive noise restarts the calibration window. LCD `CALIB` changes to `LOCKED` when calibration succeeds; UART7 reports `[INS] Stationary gyro calibration locked`. Bias is then frozen so subsequent slow rotation is not learned away. Extremely slow constant yaw during startup cannot be distinguished from bias by a six-axis IMU. Temperature changes can still cause later drift.

Calibration evaluates gyro mean and gyro/accelerometer variance over the full
window. The previous 0.01 rad/s limit on every raw gyro sample prevented startup
on this board, whose stationary gyro noise was measured near 0.013 rad/s standard
deviation. Current limits in `INS_task.c` allow 0.02 rad/s gyro standard deviation
and 0.12 m/s² acceleration standard deviation; the gyro mean must remain below
0.01 rad/s on every axis. Larger instantaneous motion and outliers still reset
the window. Keep these thresholds consistent with measurements on the actual board.

Check only sensor data while commissioning:

```bash
ros2 node info /stm32h7_node
ros2 topic echo /imu/data --qos-reliability best_effort
ros2 topic hz /imu/data --qos-reliability best_effort
```

Check that timestamps advance, quaternion norm is near 1, the stationary gyro is near zero, and rotation about each physical sensor axis has the expected sign. In WSL, reduced rates may also reflect USB forwarding and the host subscriber; a 50 Hz timer is not a measured hardware throughput guarantee.

### 3.7 `/imu/debug`

This topic continues while calibrated IMU publication is suppressed. Its values
are diagnostic telemetry, not an estimator input. The fixed array indices are:

| Indices | Contents |
|---|---|
| 0 | BMI088 initialization status; zero means success |
| 1–3 | Driver read valid, INS sample valid, calibration complete |
| 4–6 | Calibration sample count, INS sequence, sample age in milliseconds |
| 7–9 | Driver acceleration X/Y/Z, m/s² including gravity |
| 10–12 | Driver gyro X/Y/Z, rad/s before the learned residual bias correction |
| 13–15 | Accelerometer scale, configured gravity norm, INS timestep in seconds |
| 16–19 | SPI HAL error, ROS time synchronized, SYSCFG PMCR, SPI2 SR |
| 20–21 | Accelerometer and gyroscope IDs observed during initialization; expected 30 and 15 |

```bash
ros2 topic echo /imu/debug --qos-reliability best_effort
```

---

## 4. QoS Compatibility Note

> [!IMPORTANT]
> `/cmd_vel`, `/cmd_mit`, `/cmd_torque`, `/motor_fb` and `/imu/data` use **Best Effort** reliability.
> When subscribing via ROS 2 CLI tools, always specify the matching QoS profile:
> ```bash
> ros2 topic echo /motor_fb --qos-reliability best_effort
> ```
