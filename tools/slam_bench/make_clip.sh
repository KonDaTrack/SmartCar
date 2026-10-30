#!/usr/bin/env bash
# Make a short MCAP/zstd clip from the recorded bag for quick SLAM checks.
# Usage: make_clip.sh <src_bag> <out_dir> [seconds] [start_offset]
set -euo pipefail

SRC="${1:?source bag}"
OUT="${2:?output bag dir}"
SECONDS_TO_RECORD="${3:-45}"
START_OFFSET="${4:-0}"
COMPRESSION_MODE="${COMPRESSION_MODE:-none}"

rm -rf "${OUT}"
mkdir -p "$(dirname "${OUT}")"

RECORD_ARGS=(--storage mcap -o "${OUT}")
if [ "${COMPRESSION_MODE}" = "file" ]; then
  RECORD_ARGS+=(--compression-mode file --compression-format zstd)
fi

ros2 bag record "${RECORD_ARGS[@]}" \
  /lslidar_point_cloud /imu/data /chassis/odom /tf_static &
REC_PID=$!
sleep 2

ros2 bag play "${SRC}" --start-offset "${START_OFFSET}" --rate 1.0 &
PLAY_PID=$!

sleep "${SECONDS_TO_RECORD}"
kill -INT "${REC_PID}" 2>/dev/null || true
wait "${REC_PID}" || true
kill -INT "${PLAY_PID}" 2>/dev/null || true
wait "${PLAY_PID}" 2>/dev/null || true

ros2 bag info "${OUT}"
