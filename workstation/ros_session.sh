#!/usr/bin/env bash
# Run only on the PC's Ubuntu (native or WSL) installation.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
exec python3 "$(dirname -- "${BASH_SOURCE[0]}")/ros_session.py" "$@"
