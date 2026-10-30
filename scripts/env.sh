#!/usr/bin/env bash
# 用法: source scripts/env.sh
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
if [ -f "$(dirname "${BASH_SOURCE[0]}")/../install/setup.bash" ]; then
  source "$(dirname "${BASH_SOURCE[0]}")/../install/setup.bash"
fi
