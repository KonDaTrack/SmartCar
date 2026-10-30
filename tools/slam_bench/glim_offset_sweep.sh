#!/usr/bin/env bash
# Sweep GLIM's imu_time_offset on a bag and compare each trajectory to the
# chassis odometry. Writes under /results/glim_sweep_<offset>.
set -euo pipefail

BAG="${1:?bag}"
CONFIG=/opt/ros/humble/share/glim/config/config_ros.json

for OFF in 0.0 0.05 -0.05; do
  sed -i -E "s/(\"imu_time_offset\": )[^,]+/\1${OFF}/" "$CONFIG"
  OUT="/results/glim_sweep_${OFF}"
  rm -rf /tmp/dump "$OUT"
  mkdir -p "$OUT"
  timeout -s INT 30 ros2 run glim_ros glim_rosbag "$BAG" \
    > "/results/glim_sweep_${OFF}.log" 2>&1 || true
  cp /tmp/dump/traj_lidar.txt "${OUT}/traj_lidar.txt"
  echo "== imu_time_offset ${OFF}"
  grep -E "IMU errors|IMU better|No-IMU errors" "/results/glim_sweep_${OFF}.log" | tail -3 || true
  python3 /ws/tools/slam_bench/compare_to_odom.py "$BAG" "${OUT}/traj_lidar.txt" | head -2
done

sed -i -E 's/("imu_time_offset": )[^,]+/\10.0/' "$CONFIG"
