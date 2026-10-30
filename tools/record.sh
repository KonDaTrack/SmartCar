#!/usr/bin/env bash
# Record the LIO sensor set to MCAP/zstd and start the time-offset monitor.
#
# Works in the dev container (repo at /ws) and in the vehicle runtime image
# (script and monitor installed in /usr/local/bin).
#
# Default topics: C16, VX1000 IMU, chassis status/odometry and TF.
# No 2D LiDARs and no camera.
#
# Environment:
#   BAGS_DIR           bag root (default /bags)
#   MIN_FREE_GB        refuse to start below this free space (default 20)
#   MAX_BAG_SIZE_GB    split files at this size, 0 disables (default 20)
#   RECORD_SECONDS     stop after N seconds, 0 = until Ctrl-C (default 0)
#   START_SYNC_MONITOR 1 = also record arrival/stamp latency CSV (default 1)
#   RECORD_TOPICS      space-separated override of the topic list
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
BAGS_DIR="${BAGS_DIR:-/bags}"
MIN_FREE_GB="${MIN_FREE_GB:-20}"
MAX_BAG_SIZE_GB="${MAX_BAG_SIZE_GB:-20}"
RECORD_SECONDS="${RECORD_SECONDS:-0}"
START_SYNC_MONITOR="${START_SYNC_MONITOR:-1}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT="${BAGS_DIR}/${STAMP}"

# ROS setup scripts reference optional variables; do not run them with nounset.
set +u
if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  # shellcheck disable=SC1090
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
fi
if [ -f /ws/install/setup.bash ]; then
  # shellcheck disable=SC1091
  source /ws/install/setup.bash
fi
set -u

mkdir -p "$BAGS_DIR"
avail_kb="$(df -Pk "$BAGS_DIR" | awk 'NR == 2 {print $4}')"
need_kb=$((MIN_FREE_GB * 1024 * 1024))
if [ "$avail_kb" -lt "$need_kb" ]; then
  echo "not enough space in $BAGS_DIR: ${avail_kb} KB available, ${need_kb} KB required" >&2
  exit 1
fi

if [ -n "${RECORD_TOPICS:-}" ]; then
  read -r -a TOPICS <<< "$RECORD_TOPICS"
else
  TOPICS=(
    /lslidar_point_cloud
    /imu/data
    /VX1000_imu
    /vehicle_status
    /odom
    /tf
    /tf_static
  )
fi

cat > "${OUT}.manifest.yaml" <<EOF
created: $(date -Iseconds)
bag: $(basename "$OUT")
ros_domain_id: ${ROS_DOMAIN_ID:-42}
config_dir: ${SMARTCAR_CONFIG_DIR:-/config}
git_sha: $(git -C /ws rev-parse --short HEAD 2>/dev/null || echo unknown)
max_bag_size_gb: ${MAX_BAG_SIZE_GB}
sync_log: $(basename "$OUT")_sync
topics:
$(printf '  - %s\n' "${TOPICS[@]}")
EOF

MON_PID=""
TIMER_PID=""
BAG_PID=""
cleanup() {
  if [ -n "$TIMER_PID" ]; then
    kill "$TIMER_PID" 2>/dev/null || true
  fi
  if [ -n "$MON_PID" ]; then
    kill -INT "$MON_PID" 2>/dev/null || true
    wait "$MON_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT

if [ "$START_SYNC_MONITOR" = "1" ] && [ -f "${SCRIPT_DIR}/sync_monitor.py" ]; then
  SYNC_LOG_DIR="${OUT}_sync" python3 "${SCRIPT_DIR}/sync_monitor.py" &
  MON_PID=$!
fi

BAG_ARGS=(
  --storage mcap
  --compression-mode file
  --compression-format zstd
  -o "$OUT"
)
if [ "$MAX_BAG_SIZE_GB" -gt 0 ]; then
  BAG_ARGS+=(--max-bag-size $((MAX_BAG_SIZE_GB * 1024 * 1024 * 1024)))
fi

echo "Recording to $OUT"
echo "Manifest: ${OUT}.manifest.yaml"
if [ "$MON_PID" != "" ]; then
  echo "Sync monitor: ${OUT}_sync"
else
  echo "Sync monitor: disabled"
fi

ros2 bag record "${BAG_ARGS[@]}" "${TOPICS[@]}" &
BAG_PID=$!

trap 'kill -INT "$BAG_PID" 2>/dev/null || true' INT TERM

if [ "$RECORD_SECONDS" -gt 0 ]; then
  ( sleep "$RECORD_SECONDS"; kill -INT "$BAG_PID" 2>/dev/null || true ) &
  TIMER_PID=$!
fi

wait "$BAG_PID" || true
echo "Bag finished: $OUT"
