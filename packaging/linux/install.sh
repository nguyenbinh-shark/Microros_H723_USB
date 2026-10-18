#!/usr/bin/env bash
# Install RPBot Studio and DAPLink Flasher for the current user (no sudo needed).
#   ./install.sh              copy the apps to ~/.local/share/rpbot and add them to the app menu
#   ./install.sh --udev       also let desktop users open CMSIS-DAP probes without sudo (asks for sudo)
#   ./install.sh --uninstall  remove the apps and menu entries
set -euo pipefail
src="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
data="${XDG_DATA_HOME:-$HOME/.local/share}"
dest="$data/rpbot"
apps="$data/applications"
rules=/etc/udev/rules.d/60-dap-flasher.rules

entry() {  # file, name, executable, icon, comment, window class
    cat > "$apps/$1" <<EOF
[Desktop Entry]
Type=Application
Name=$2
Comment=$5
Exec="$3"
Icon=$4
Terminal=false
Categories=Development;Engineering;
StartupWMClass=$6
EOF
}

if [ "${1:-}" = --uninstall ]; then
    rm -rf "$dest" "$apps/rpbot-studio.desktop" "$apps/dap-flasher.desktop"
    [ ! -f "$rules" ] || echo "The udev rule stays; remove it with: sudo rm $rules"
    echo "Removed RPBot Studio and DAPLink Flasher. Settings stay in ~/.local/state/RPBotConsole and ~/.config/dap_flasher."
    exit 0
fi

mkdir -p "$dest" "$apps"
install -m 755 "$src/RPBotStudio" "$src/DAPLinkFlasher" "$dest/"
install -m 644 "$src/rpbot-studio.png" "$src/dap-flasher.png" "$dest/"
entry rpbot-studio.desktop "RPBot Studio" "$dest/RPBotStudio" "$dest/rpbot-studio.png" \
    "Mapping, Navigation and diagnostics for RPBot" RPBotStudio
entry dap-flasher.desktop "DAPLink Flasher" "$dest/DAPLinkFlasher" "$dest/dap-flasher.png" \
    "Flash STM32H723 firmware over CMSIS-DAP" DAPLinkFlasher
command -v update-desktop-database >/dev/null && update-desktop-database "$apps" || true

if [ "${1:-}" = --udev ]; then
    sudo install -m 644 "$src/60-dap-flasher.rules" "$rules"
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    echo "udev rule installed; unplug and replug the probe."
fi

[ -f /opt/ros/jazzy/setup.bash ] || echo "Note: RPBot Studio needs ROS 2 Jazzy with RViz: sudo apt install ros-jazzy-desktop"
command -v ssh >/dev/null || echo "Note: RPBot Studio needs ssh: sudo apt install openssh-client"
command -v openocd >/dev/null || echo "Note: DAPLink Flasher needs OpenOCD: sudo apt install openocd"
[ -f "$rules" ] || echo "Note: run ./install.sh --udev once so DAPLink Flasher can open the probe without sudo."
echo "Installed in $dest. Open RPBot Studio or DAPLink Flasher from the app menu."
