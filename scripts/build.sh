#!/usr/bin/env bash
# 增量构建。用法：
#   ./scripts/build.sh ros_control_msgs
#   ./scripts/build.sh ros_control_msgs ros_control_hardware
set -euo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${WORKSPACE_DIR}"

if [[ -z "${ROS_DISTRO:-}" ]]; then
  echo "请先 source /opt/ros/<distro>/setup.bash" >&2
  exit 1
fi

if [[ $# -eq 0 ]]; then
  echo "用法: $0 <package> [package...]" >&2
  exit 1
fi

PACKAGES=()
for pkg in "$@"; do
  PACKAGES+=(--packages-select "${pkg}")
done

colcon build --symlink-install --event-handlers console_direct+ "${PACKAGES[@]}"
echo
echo "构建完成。下一步: source ${WORKSPACE_DIR}/install/setup.bash"
