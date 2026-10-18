#!/usr/bin/env bash
# Build the Linux package of RPBot Studio and DAPLink Flasher on Ubuntu 24.04 (native or WSL).
# Needs: sudo apt install python3-tk python3-venv binutils
# Output: dist/linux/RPBot-Linux-<arch>.tar.gz (install with ./install.sh inside it).
# The binaries need glibc 2.39 or newer, i.e. Ubuntu 24.04 or later.
set -euo pipefail
here="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo="$(cd -- "$here/../.." && pwd)"
venv="${RPBOT_BUILD_VENV:-$HOME/.cache/rpbot-build-venv}"
# Build and stage on the Linux file system so tar keeps real file modes (not /mnt/<drive>).
work="$(mktemp -d "${TMPDIR:-/tmp}/rpbot-linux-build.XXXXXX")"
trap 'rm -rf "$work"' EXIT
name="RPBot-Linux-$(uname -m)"
stage="$work/$name"

python3 -c 'import tkinter' 2>/dev/null || {
    echo 'Missing Tk: sudo apt install python3-tk python3-venv' >&2; exit 1; }
if [ ! -x "$venv/bin/pyinstaller" ]; then
    python3 -m venv "$venv"
    "$venv/bin/pip" install -q pyinstaller==6.22.3 hidapi==0.15.0
fi

mkdir -p "$stage"
for spec in "$repo/workstation/RPBotStudio.spec" "$repo/tools/DAPLinkFlasher.spec"; do
    "$venv/bin/python" -B -m PyInstaller --noconfirm --log-level WARN \
        --distpath "$stage" --workpath "$work/pyinstaller" "$spec"
done
install -m 644 "$repo/workstation/assets/rpbot-studio-256.png" "$stage/rpbot-studio.png"
install -m 644 "$repo/tools/assets/dap-flasher-256.png" "$stage/dap-flasher.png"
install -m 644 "$here/60-dap-flasher.rules" "$here/README.txt" "$stage/"
install -m 755 "$here/install.sh" "$stage/"

mkdir -p "$repo/dist/linux"
tar -C "$work" -czf "$work/$name.tar.gz" "$name"
cp "$work/$name.tar.gz" "$repo/dist/linux/"
ls -l "$stage"
sha256sum "$repo/dist/linux/$name.tar.gz"
