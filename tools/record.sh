#!/usr/bin/env bash
# Record the SmartCar sensor topics on the development machine.
# Run inside the dev/record container (the repo is mounted at /ws).
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
BAGS_DIR="${BAGS_DIR:-/bags}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT="${BAGS_DIR}/${STAMP}"

# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [ -f /ws/install/setup.bash ]; then
  # shellcheck disable=SC1091
  source /ws/install/setup.bash
fi

mkdir -p "$BAGS_DIR"

TOPICS=(
  /lslidar_point_cloud
  /scan_left
  /scan_right
  /VX1000_imu
  /VX1000_gnss
  /vehicle_status
  /tf
  /tf_static
)

cat > "${OUT}.manifest.yaml" <<EOF
created: $(date -Iseconds)
bag: $(basename "$OUT")
ros_domain_id: ${ROS_DOMAIN_ID:-42}
config_dir: ${SMARTCAR_CONFIG_DIR:-/config}
git_sha: $(git -C /ws rev-parse --short HEAD 2>/dev/null || echo unknown)
topics:
$(printf '  - %s\n' "${TOPICS[@]}")
EOF

echo "Recording to $OUT"
echo "Manifest: ${OUT}.manifest.yaml"

# If a sensor driver publishes best-effort QoS, add:
#   --qos-profile-overrides-path /config/rosbag_qos.yaml
exec ros2 bag record \
  --storage mcap \
  --compression-mode file \
  --compression-format zstd \
  -o "$OUT" \
  "${TOPICS[@]}"
