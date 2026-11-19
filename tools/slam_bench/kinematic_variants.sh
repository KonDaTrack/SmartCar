#!/usr/bin/env bash
# Run Kinematic-ICP deskew/regularization variants on one bag and report
# path length against the bag's chassis odometry.
set -euo pipefail

BAG="${1:?bag}"
PREFIX="${2:-/results/kinematic_variants}"

run_variant() {
  local name="$1" deskew="$2" adaptive="$3"
  local out="${PREFIX}/${name}"
  mkdir -p "$out"
  ros2 run kinematic_icp kinematic_icp_offline_node --ros-args \
    -p lidar_topic:=/lslidar_point_cloud \
    -p bag_filename:="$BAG" \
    -p output_dir:="$out" \
    -p use_2d_lidar:=false \
    -p lidar_odom_frame:=odom_lidar \
    -p wheel_odom_frame:=chassis_odom \
    -p base_frame:=base_link \
    -p publish_odom_tf:=false \
    -p invert_odom_tf:=true \
    -p tf_timeout:=0.1 \
    -p deskew:="$deskew" \
    -p use_adaptive_odometry_regularization:="$adaptive" \
    -p fixed_regularization:=0.0 >/dev/null 2>&1
  echo "== ${name} deskew=${deskew} adaptive_reg=${adaptive}"
  python3 /ws/tools/slam_bench/compare_to_odom.py "$BAG" \
    "$out/$(basename "$BAG")_kinematic_icp_poses_tum.txt" | head -3
}

run_variant deskew_off_reg_on false true
run_variant deskew_on_reg_off true false
