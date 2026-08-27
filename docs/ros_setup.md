# Host PC & ROS 2 Environment Setup

This guide details how to set up the host computer (Raspberry Pi, Jetson, Linux PC, or WSL2) to communicate with the STM32H723 micro-ROS robot node.

---

## 1. Installing micro-ROS Agent (ROS 2 Jazzy)

### Option A — Native Colcon Build (Recommended for SBCs)

```bash
# 1. Create a dedicated workspace
mkdir -p ~/microros_ws/src
cd ~/microros_ws/src

# 2. Clone the micro-ROS Agent package for ROS 2 Jazzy
git clone -b jazzy https://github.com/micro-ROS/micro_ros_agent.git

# 3. Resolve dependencies and build
cd ~/microros_ws
source /opt/ros/jazzy/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
```

### Option B — Running via Docker

```bash
docker run -it --rm --net=host --privileged \
  -v /dev:/dev \
  microros/micro-ros-agent:jazzy \
  serial --dev /dev/stm32_robot
```

---

## 2. Automated Host Deployment (UDEV + Systemd)

The `deploy/` directory provides an automated installer that locks the USB port to `/dev/stm32_robot` and sets up a background service.

```bash
cd deploy/
sudo ./install_host.sh
```

### What this script configures:
1. **UDEV Rule (`/etc/udev/rules.d/99-stm32-robot.rules`):** Matches ST USB CDC (`VID=0483`, `PID=5740`) and creates the persistent symlink `/dev/stm32_robot`.
2. **User Permissions:** Adds your non-root user to the `dialout` group.
3. **Systemd Service (`/etc/systemd/system/micro_ros.service`):** Automatically starts `micro_ros_agent` on system boot, with auto-restart on USB reconnects.

---

## 3. WSL2 Integration via `usbipd` (Windows Users)

If you are developing inside WSL2 on Windows:

1. Open PowerShell on Windows (as Administrator):
   ```powershell
   # List connected USB devices
   usbipd list
   # Find STM32 USB CDC device (e.g., busid 2-4) and bind it:
   usbipd bind --busid 2-4
   # Attach to WSL2:
   usbipd attach --wsl --busid 2-4
   ```
2. Inside WSL2, verify that `/dev/ttyACM0` or `/dev/stm32_robot` appears:
   ```bash
   ls -la /dev/ttyACM* /dev/stm32_robot
   ```

---

## 4. Verifying Communication

Once the micro-ROS agent is running and connected to STM32:

### Check Node Discovery
```bash
source /opt/ros/jazzy/setup.bash
ros2 node list
```
*Expected Output:*
```text
/stm32h7_node
```

### Echo Feedback Topic (Best Effort QoS)
```bash
ros2 topic echo /motor_fb --qos-reliability best_effort
```

### Echo Status Topic (Reliable QoS)
```bash
ros2 topic echo /motor_status
```
