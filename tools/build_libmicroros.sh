#!/usr/bin/env bash
# Rebuild micro-ROS static library using official Docker container
# Usage: ./tools/build_libmicroros.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

if ! command -v docker >/dev/null 2>&1; then
    echo "[ERROR] Docker is not installed or not in PATH." >&2
    echo "Please install Docker to build micro-ROS library locally." >&2
    exit 1
fi

echo "[INFO] Building micro-ROS static library for ROS 2 Jazzy..."
echo "[INFO] Project root: ${PROJECT_ROOT}"

docker run -it --rm \
    -v "${PROJECT_ROOT}:/project" \
    --env MICROROS_LIBRARY_FOLDER=micro_ros_stm32cubemx_utils/microros_static_library \
    microros/micro_ros_static_library_builder:jazzy

echo "[SUCCESS] Build completed. Static library is in micro_ros_stm32cubemx_utils/microros_static_library/libmicroros"
