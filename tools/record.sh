#!/usr/bin/env bash
# Record the LIO sensor set to MCAP and start the time-offset monitor.
#
# Works in the dev container (repo at /ws) and in the vehicle runtime image
# (script and monitor installed in /usr/local/bin).
#
# Default topics: C16, VX1000 INS position/status, VX1000 IMU and TF.
# No 2D LiDARs and no chassis topics. Set RECORD_CAMERA=1 to append the
# D435i color/depth/infra streams (its IMU is disabled in the launch file).
#
# Environment:
#   BAGS_DIR           bag root (default /bags)
#   MIN_FREE_GB        refuse to start below this free space (default 20)
#   MAX_BAG_SIZE_GB    split files at this size, 0 disables (default 20)
#   MAX_CACHE_SIZE_MB  rosbag2 cache per topic (default 128); raising it
#                      prevents small high-rate VX1000 messages from being
#                      dropped while camera images are written
#   COMPRESSION_MODE   none (default), file or message; file uses zstd
#   QOS_PROFILE        subscription QoS overrides (default /config/record_qos.yaml)
#   RECORD_SECONDS     stop after N seconds, 0 = until Ctrl-C (default 0)
#   START_SYNC_MONITOR 1 = also record arrival/stamp latency CSV (default 1)
#   RECORD_TOPICS      space- or newline-separated topic list override
#   RECORD_CAMERA      1 = append D435i color/depth/infra topics (no IMU)
#   BAG_NAME           optional output bag name (default: timestamp)
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
BAGS_DIR="${BAGS_DIR:-/bags}"
MIN_FREE_GB="${MIN_FREE_GB:-20}"
MAX_BAG_SIZE_GB="${MAX_BAG_SIZE_GB:-20}"
MAX_CACHE_SIZE_MB="${MAX_CACHE_SIZE_MB:-128}"
COMPRESSION_MODE="${COMPRESSION_MODE:-none}"
QOS_PROFILE="${QOS_PROFILE:-/config/record_qos.yaml}"
RECORD_SECONDS="${RECORD_SECONDS:-0}"
START_SYNC_MONITOR="${START_SYNC_MONITOR:-1}"
RECORD_CAMERA="${RECORD_CAMERA:-0}"
BAG_NAME="${BAG_NAME:-}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STAMP="$(date +%Y%m%d_%H%M%S)"
if [ -n "$BAG_NAME" ]; then
  OUT="${BAGS_DIR}/${BAG_NAME}"
else
  OUT="${BAGS_DIR}/${STAMP}"
fi

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
  # Accept both space-separated and multi-line topic lists. Plain `read -a`
  # stops at the first newline and would silently record only one topic.
  read -r -a TOPICS <<< "$(printf '%s' "$RECORD_TOPICS" | tr '\n' ' ')"
else
  TOPICS=(
    /lslidar_point_cloud
    /VX1000_ins
    /VX1000_cov
    /VX1000_stat
    /imu/data
    /VX1000_imu
    /tf
    /tf_static
  )
  if [ "$RECORD_CAMERA" = "1" ]; then
    TOPICS+=(
      /camera/camera/color/image_raw
      /camera/camera/color/camera_info
      /camera/camera/depth/image_rect_raw
      /camera/camera/depth/camera_info
      /camera/camera/infra1/image_rect_raw
      /camera/camera/infra1/camera_info
      /camera/camera/infra2/image_rect_raw
      /camera/camera/infra2/camera_info
    )
  fi
fi

cat > "${OUT}.manifest.yaml" <<EOF
created: $(date -Iseconds)
bag: $(basename "$OUT")
ros_domain_id: ${ROS_DOMAIN_ID:-42}
config_dir: ${SMARTCAR_CONFIG_DIR:-/config}
git_sha: $(git -C /ws rev-parse --short HEAD 2>/dev/null || echo unknown)
max_bag_size_gb: ${MAX_BAG_SIZE_GB}
max_cache_size_mb: ${MAX_CACHE_SIZE_MB}
compression_mode: ${COMPRESSION_MODE}
qos_profile: ${QOS_PROFILE}
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
  --max-cache-size $((MAX_CACHE_SIZE_MB * 1024 * 1024))
  -o "$OUT"
)
if [ "$COMPRESSION_MODE" != "none" ]; then
  BAG_ARGS+=(--compression-mode "$COMPRESSION_MODE" --compression-format zstd)
fi
if [ -f "$QOS_PROFILE" ]; then
  BAG_ARGS+=(--qos-profile-overrides-path "$QOS_PROFILE")
fi
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

FINALIZING=0
stop_bag() {
  if [ "$FINALIZING" = "1" ]; then
    return
  fi
  FINALIZING=1
  echo "Recorder stopping; finalizing bag. Do not stop the container until BAG READY."
  kill -INT "$BAG_PID" 2>/dev/null || true
}
trap stop_bag INT TERM

if [ "$RECORD_SECONDS" -gt 0 ]; then
  ( sleep "$RECORD_SECONDS"; kill -INT "$BAG_PID" 2>/dev/null || true ) &
  TIMER_PID=$!
fi

wait "$BAG_PID" || true
FINALIZING=1
echo "Finalizing $OUT ..."

# File compression runs at shutdown and can be interrupted by a short docker
# stop timeout. If metadata was not written, reconstruct it from the MCAP file
# before declaring the bag ready.
if [ ! -f "$OUT/metadata.yaml" ]; then
  echo "metadata.yaml missing; reindexing MCAP"
  ros2 bag reindex -s mcap "$OUT" || true
fi

if [ -f "$OUT/metadata.yaml" ] && ros2 bag info "$OUT"; then
  touch "${OUT}.ready"
  echo "BAG READY: $OUT"
else
  echo "ERROR: bag finalization failed; raw data may remain in $OUT" >&2
  exit 1
fi
