#!/usr/bin/env bash
# Rebuild the vendored Kinematic-ICP ROS 2 wrapper inside the bench container.
set -euo pipefail
ROS_DISTRO="${ROS_DISTRO:-humble}"
KIN_WS=/opt/kinematic_icp_ws

rm -rf "${KIN_WS}/src/kinematic_icp_repo"
mkdir -p "${KIN_WS}/src"
cp -a /ws/src/vendor/kinematic_icp "${KIN_WS}/src/kinematic_icp_repo"

set +u
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u
cd "${KIN_WS}"
colcon build --packages-select kinematic_icp \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
