#!/usr/bin/env bash
# Source the ROS and benchmark workspaces, then exec the given command.
# Kept in a script so the host-side docker exec does not need to source ROS.
set -e
set +u
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
if [ -f /opt/kinematic_icp_ws/install/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/kinematic_icp_ws/install/setup.bash
fi
set -u
exec "$@"
