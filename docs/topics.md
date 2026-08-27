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
| `/cmd_vel` | Subscriber | `geometry_msgs/msg/Twist` | **Best Effort** | Up to 50 Hz | Target velocity command for differential drive |
| `/motor_enable` | Subscriber | `std_msgs/msg/Bool` | **Reliable** | On-demand | Enable (`true`) or Disable (`false`) motor outputs |
| `/motor_fb` | Publisher | `sensor_msgs/msg/JointState` | **Best Effort** | **50 Hz** | Position, velocity, and torque feedback from wheels |
| `/motor_status` | Publisher | `std_msgs/msg/UInt8` | **Reliable** | **5 Hz** | System health, CAN status, and timeout bitmask |

---

## 3. Topic Specifications

### 3.1 `/cmd_vel` (Velocity Command)

- **Type:** `geometry_msgs/msg/Twist`
- **Subscribed Fields:**
  - `linear.x` (float64, interpreted as float32): Forward linear velocity in **$\text{m/s}$**.
  - `angular.z` (float64, interpreted as float32): Yaw rotational velocity in **$\text{rad/s}$**.
- **Differential Kinematics:**
  $$\omega_L = \frac{v_x - \frac{W}{2}\omega_z}{R}, \quad \omega_R = \frac{v_x + \frac{W}{2}\omega_z}{R}$$
  where $R = \text{WHEEL\_RADIUS} = 0.05\text{ m}$ and $W = \text{WHEEL\_BASE} = 0.30\text{ m}$.
- **Safety Timeout:** If no `/cmd_vel` message is received for `CMD_VEL_TIMEOUT_MS` (500 ms), the firmware resets wheel target velocities to `0.0 rad/s` and raises the `MSTAT_CMD_TIMEOUT` status flag.

### 3.2 `/motor_enable` (Motor State Control)

- **Type:** `std_msgs/msg/Bool`
- **Data:**
  - `true`: Sends extended CAN enable frames to Left and Right RobStride motors, puts them into Run Mode.
  - `false`: Sends CAN stop frames to both motors and cuts torque.
- **Initial State on Boot:** `false` (Motors start disabled for safety).

### 3.3 `/motor_fb` (Wheel Odometry & Joint Feedback)

- **Type:** `sensor_msgs/msg/JointState`
- **Publish Rate:** 50 Hz (every 20 ms in `defaultTask`)
- **Header:**
  - `header.stamp`: Synchronized ROS 2 epoch timestamp (via micro-ROS time agent sync).
  - `header.frame_id`: `"base_link"`
- **Arrays (Size 2):**
  - `name`: `["left_wheel_joint", "right_wheel_joint"]`
  - `position`: `[pos_left_rad, pos_right_rad]` (continuous unwrapped radians)
  - `velocity`: `[vel_left_rad_s, vel_right_rad_s]` (rad/s)
  - `effort`: `[torq_left_nm, torq_right_nm]` (measured torque in Nm)

### 3.4 `/motor_status` (Status Bitmask)

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
| `5` | `0x20` | `MSTAT_CMD_TIMEOUT`| `/cmd_vel` command watchdog timeout (> 500 ms idle) |
| `6..7` | `0xC0` | *Reserved* | Reserved for future expansion |

---

## 4. QoS Compatibility Note

> [!IMPORTANT]
> `/cmd_vel` and `/motor_fb` use **Best Effort** reliability to avoid buffering outdated motion commands.
> When subscribing via ROS 2 CLI tools, always specify the matching QoS profile:
> ```bash
> ros2 topic echo /motor_fb --qos-reliability best_effort
> ```

