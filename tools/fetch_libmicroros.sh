#!/usr/bin/env bash
# Fetch prebuilt micro-ROS static library for STM32H723 (ROS 2 Jazzy)
# Usage: ./tools/fetch_libmicroros.sh
# Override local tarball: LIBMICROROS_TARBALL=/path/to/archive.tar.gz ./tools/fetch_libmicroros.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

VERSION="v1.0.0"
TARBALL_NAME="libmicroros-jazzy-stm32h723-${VERSION}.tar.gz"
EXPECTED_SHA256="3c4773d7baeead01defede2536e031c8b87c185da0860f0021021899b4db66bc"
RELEASE_URL="https://github.com/nguyenbinh-shark/ros_h7_usb/releases/download/${VERSION}/${TARBALL_NAME}"

DEST_DIR="${PROJECT_ROOT}/micro_ros_stm32cubemx_utils/microros_static_library"
TARGET_LIB="${DEST_DIR}/libmicroros/libmicroros.a"

if [ -f "${TARGET_LIB}" ]; then
    echo "[INFO] micro-ROS library already present at: ${TARGET_LIB}"
    read -r -p "Do you want to re-download and overwrite? [y/N] " response || response="N"
    case "$response" in
        [yY][eE][sS]|[yY])
            echo "[INFO] Proceeding to overwrite..."
            ;;
        *)
            echo "[INFO] Skipping download. Existing library kept."
            exit 0
            ;;
    esac
fi

mkdir -p "${DEST_DIR}"
TMP_DIR="$(mktemp -d 2>/dev/null || mktemp -d -t 'microros_fetch')"
trap 'rm -rf "${TMP_DIR}"' EXIT

ARCHIVE_PATH=""

if [ -n "${LIBMICROROS_TARBALL:-}" ]; then
    echo "[INFO] Using local tarball: ${LIBMICROROS_TARBALL}"
    if [ ! -f "${LIBMICROROS_TARBALL}" ]; then
        echo "[ERROR] Local tarball not found at: ${LIBMICROROS_TARBALL}" >&2
        exit 1
    fi
    ARCHIVE_PATH="${LIBMICROROS_TARBALL}"
else
    echo "[INFO] Downloading ${TARBALL_NAME} from GitHub Release ${VERSION}..."
    ARCHIVE_PATH="${TMP_DIR}/${TARBALL_NAME}"
    if command -v curl >/dev/null 2>&1; then
        curl -fSL --progress-bar -o "${ARCHIVE_PATH}" "${RELEASE_URL}"
    elif command -v wget >/dev/null 2>&1; then
        wget -q --show-progress -O "${ARCHIVE_PATH}" "${RELEASE_URL}"
    else
        echo "[ERROR] Neither curl nor wget found. Please install either or set LIBMICROROS_TARBALL." >&2
        exit 1
    fi
fi

echo "[INFO] Verifying SHA256 checksum..."
if command -v sha256sum >/dev/null 2>&1; then
    ACTUAL_SHA256="$(sha256sum "${ARCHIVE_PATH}" | awk '{print $1}')"
elif command -v shasum >/dev/null 2>&1; then
    ACTUAL_SHA256="$(shasum -a 256 "${ARCHIVE_PATH}" | awk '{print $1}')"
else
    ACTUAL_SHA256="$(python3 -c "import hashlib; print(hashlib.sha256(open('${ARCHIVE_PATH}', 'rb').read()).hexdigest())" 2>/dev/null || python -c "import hashlib; print(hashlib.sha256(open('${ARCHIVE_PATH}', 'rb').read()).hexdigest())")"
fi

if [ "${ACTUAL_SHA256,,}" != "${EXPECTED_SHA256,,}" ]; then
    echo "[ERROR] SHA256 mismatch!" >&2
    echo "  Expected: ${EXPECTED_SHA256}" >&2
    echo "  Actual:   ${ACTUAL_SHA256}" >&2
    exit 1
fi
echo "[INFO] Checksum verified: OK"

echo "[INFO] Extracting archive to ${DEST_DIR}..."
if tar --help 2>&1 | grep -q -- '--force-local'; then
    tar --force-local -xzf "${ARCHIVE_PATH}" -C "${DEST_DIR}"
else
    tar -xzf "${ARCHIVE_PATH}" -C "${DEST_DIR}"
fi

if [ ! -f "${TARGET_LIB}" ]; then
    echo "[ERROR] Extraction failed: ${TARGET_LIB} not found." >&2
    exit 1
fi

echo "[SUCCESS] micro-ROS library installed successfully at ${DEST_DIR}/libmicroros"
