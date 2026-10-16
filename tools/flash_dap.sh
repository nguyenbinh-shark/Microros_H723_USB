#!/usr/bin/env bash
# Flash or debug the STM32H723 through a CMSIS-DAP probe (e.g. wireless DAPLink) with OpenOCD.
# Usage: ./tools/flash_dap.sh [FILE]     # program FILE (default: build/microros_H7.elf), verify, reset
#        ./tools/flash_dap.sh --server   # start a GDB server on localhost:3333 for debugging
# .elf/.hex are written to their own addresses, .bin to 0x08000000.
# OpenOCD lookup: $OPENOCD, then openocd on PATH, then the copy bundled with STM32CubeIDE.
# Override scripts dir: OPENOCD_SCRIPTS=/path/to/scripts

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
CUBEIDE_PLUGINS="/c/ST/STM32CubeIDE_*/STM32CubeIDE/plugins"

latest() {
    ls -d $1 2>/dev/null | sort -V | tail -n 1
}

# Native Windows OpenOCD cannot read MSYS paths such as /p/... inside -c commands.
native_path() {
    if command -v cygpath >/dev/null 2>&1; then
        cygpath -m "$1"
    else
        printf '%s\n' "$1"
    fi
}

OPENOCD="${OPENOCD:-}"
SCRIPTS="${OPENOCD_SCRIPTS:-}"
if [ -z "$OPENOCD" ]; then
    if command -v openocd >/dev/null 2>&1; then
        OPENOCD="$(command -v openocd)"
    else
        OPENOCD="$(latest "$CUBEIDE_PLUGINS/com.st.stm32cube.ide.mcu.externaltools.openocd.*/tools/bin/openocd.exe")"
        if [ -z "$SCRIPTS" ]; then
            SCRIPTS="$(latest "$CUBEIDE_PLUGINS/com.st.stm32cube.ide.mcu.debug.openocd_*/resources/openocd/st_scripts")"
        fi
    fi
fi
if [ -z "$OPENOCD" ] || [ ! -x "$OPENOCD" ]; then
    echo "[ERROR] OpenOCD not found. Install openocd or STM32CubeIDE, or set OPENOCD=/path/to/openocd." >&2
    exit 1
fi

ARGS=()
if [ -n "$SCRIPTS" ]; then
    ARGS+=(-s "$(native_path "$SCRIPTS")")
fi
ARGS+=(-f "$(native_path "$SCRIPT_DIR/openocd_dap.cfg")")

if [ "${1:-}" = "--server" ]; then
    echo "[INFO] GDB server on localhost:3333 (Ctrl+C to stop)"
    exec "$OPENOCD" "${ARGS[@]}"
fi

FILE="${1:-$PROJECT_ROOT/build/microros_H7.elf}"
if [ ! -f "$FILE" ]; then
    echo "[ERROR] Firmware file not found: $FILE (run make first)" >&2
    exit 1
fi
FILE="$(native_path "$FILE")"

case "$FILE" in
    *.bin) PROGRAM="program {$FILE} 0x08000000 verify reset exit" ;;
    *)     PROGRAM="program {$FILE} verify reset exit" ;;
esac

echo "[INFO] Flashing $FILE via CMSIS-DAP"
exec "$OPENOCD" "${ARGS[@]}" -c "$PROGRAM"
