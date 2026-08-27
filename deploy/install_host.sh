#!/usr/bin/env bash
# Auto-deploy script for host PC / Raspberry Pi / Jetson running ROS 2 Jazzy
# Sets up UDEV rule for STM32 USB CDC device and installs systemd service.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"

echo "================================================"
echo "   Auto-Deploy Script cho Host PC / Raspberry   "
echo "================================================"

if [ "${EUID:-$(id -u)}" -ne 0 ]; then
    echo "[ERROR] Please run this script with root privileges: sudo ./install_host.sh" >&2
    exit 1
fi

ROBOT_USER="${SUDO_USER:-$USER}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
AGENT_WS="${AGENT_WS:-/home/${ROBOT_USER}/microros_ws/install/setup.bash}"

echo "[INFO] Target user: ${ROBOT_USER}"
echo "[INFO] ROS setup:   ${ROS_SETUP}"
echo "[INFO] Agent ws:    ${AGENT_WS}"

echo "[1/4] Copying UDEV rules for STM32..."
cp "${SCRIPT_DIR}/99-stm32-robot.rules" /etc/udev/rules.d/
udevadm control --reload-rules
udevadm trigger
echo " => OK: STM32 will be accessible at /dev/stm32_robot"

echo "[2/4] Ensuring ${ROBOT_USER} is in dialout group..."
usermod -aG dialout "${ROBOT_USER}" || true
echo " => OK: User added to dialout group"

echo "[3/4] Configuring Systemd Service for micro-ROS Agent..."
sed \
    -e "s|@USER@|${ROBOT_USER}|g" \
    -e "s|@ROS_SETUP@|${ROS_SETUP}|g" \
    -e "s|@AGENT_WS@|${AGENT_WS}|g" \
    "${SCRIPT_DIR}/micro_ros.service.in" > /etc/systemd/system/micro_ros.service

systemctl daemon-reload
systemctl enable micro_ros.service
echo " => OK: micro_ros.service enabled on boot"

echo "[4/4] Starting service..."
systemctl restart micro_ros.service || true
systemctl status micro_ros.service --no-pager || true

echo "================================================"
echo " HOAN TAT! Robot da o trang thai Plug & Play.   "
echo "================================================"

