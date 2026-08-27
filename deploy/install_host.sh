#!/bin/bash
echo "================================================"
echo "   Auto-Deploy Script cho Host PC / Raspberry   "
echo "================================================"

if [ "$EUID" -ne 0 ]; then
  echo "[L?I] Vui lòng ch?y script này b?ng quy?n root (sudo ./install_host.sh)"
  exit
fi

echo "[1/3] Copy Udev rules cho STM32..."
cp 99-stm32-robot.rules /etc/udev/rules.d/
udevadm control --reload-rules
udevadm trigger
echo " => Xong! Board STM32 s? luôn du?c nh?n là /dev/stm32_robot"

echo "[2/3] Cài d?t Systemd Service cho micro-ROS Agent..."
cp micro_ros.service /etc/systemd/system/
systemctl daemon-reload
systemctl enable micro_ros.service
echo " => Xong! Service micro_ros dã du?c n?p và kích ho?t t? ch?y khi boot."

echo "[3/3] Kh?i d?ng Service ngay bây gi?..."
systemctl restart micro_ros.service
systemctl status micro_ros.service --no-pager

echo "================================================"
echo " HOÀN T?T! Robot dã ? tr?ng thái Plug & Play.   "
echo "================================================"
