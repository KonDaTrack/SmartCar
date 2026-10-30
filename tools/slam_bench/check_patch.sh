#!/usr/bin/env bash
# Verify that the benchmark patch still applies to the upstream Kinematic-ICP.
set -euo pipefail
rm -rf /tmp/kinematic_icp_check
git clone --depth 1 https://github.com/PRBonn/kinematic-icp.git \
  /tmp/kinematic_icp_check
patch -p1 -d /tmp/kinematic_icp_check \
  < /ws/tools/slam_bench/patches/kinematic_icp_ros_bench.patch
