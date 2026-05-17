#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

cd "${WORKSPACE_DIR}"
source /opt/ros/humble/setup.bash

if [[ -f "${WORKSPACE_DIR}/venv/bin/activate" ]]; then
  source "${WORKSPACE_DIR}/venv/bin/activate"
fi

if [[ ! -f "${WORKSPACE_DIR}/install/setup.bash" ]]; then
  echo "Workspace is not built yet. Run: colcon build" >&2
  exit 1
fi

source "${WORKSPACE_DIR}/install/setup.bash"
set -u

exec ros2 launch dual_arm_teleop full_teleop.launch.py "$@"
