#!/usr/bin/env bash
# Camera intrinsic calibration with camera_calibration in the dev container.
#
# The calibration GUI runs on the development host. The vehicle only publishes
# the camera topics over DDS; no GUI or calibration package is installed on the
# vehicle runtime.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
. "${SCRIPT_DIR}/common.sh"

SIZE="8x6"
SQUARE="0.025"
TOPIC="/camera/camera/color/image_raw"
CAMERA="/camera/camera/color"
STEREO=0
LEFT="/camera/camera/infra1/image_rect_raw"
RIGHT="/camera/camera/infra2/image_rect_raw"
LEFT_CAMERA="/camera/camera/infra1"
RIGHT_CAMERA="/camera/camera/infra2"

usage() {
  cat <<'EOF'
Usage: ./scripts/calib-intrinsics.sh [options]

Options:
  --size NxM              checkerboard inner corners (default: 8x6)
  --square METERS         square size in metres (default: 0.025)
  --topic TOPIC           monocular image topic
                          (default: /camera/camera/color/image_raw)
  --camera NAMESPACE      camera_info namespace
                          (default: /camera/camera/color)
  --stereo                stereo calibration (infra1/infra2 by default)
  --left TOPIC            left image topic for --stereo
  --right TOPIC           right image topic for --stereo
  --left-camera NS        left camera_info namespace for --stereo
  --right-camera NS       right camera_info namespace for --stereo

Examples:
  ./scripts/calib-intrinsics.sh --size 9x6 --square 0.025
  ./scripts/calib-intrinsics.sh \
    --topic /camera/camera/infra1/image_rect_raw \
    --camera /camera/camera/infra1
  ./scripts/calib-intrinsics.sh --stereo

The GUI runs in the `calib` Compose service with X11. Move the board until the
X/Y/Size/Skew bars are green, click CALIBRATE, then SAVE. The resulting
/tmp/calibrationdata.tar.gz is copied to config/calibration/intrinsics/.
EOF
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    --size) SIZE="$2"; shift 2 ;;
    --square) SQUARE="$2"; shift 2 ;;
    --topic) TOPIC="$2"; shift 2 ;;
    --camera) CAMERA="$2"; shift 2 ;;
    --stereo) STEREO=1; shift ;;
    --left) LEFT="$2"; shift 2 ;;
    --right) RIGHT="$2"; shift 2 ;;
    --left-camera) LEFT_CAMERA="$2"; shift 2 ;;
    --right-camera) RIGHT_CAMERA="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

HOST_USER="$(id -un)"
xhost +si:localuser:"${HOST_USER}" >/dev/null 2>&1 || true

mkdir -p "${ROOT}/config/calibration/intrinsics"

ARGS=(--size "$SIZE" --square "$SQUARE" --no-service-check)
if [ "$STEREO" = "1" ]; then
  echo "Stereo calibration: left=$LEFT right=$RIGHT"
  ARGS+=(
    "left:=$LEFT"
    "right:=$RIGHT"
    "left_camera:=$LEFT_CAMERA"
    "right_camera:=$RIGHT_CAMERA"
  )
else
  echo "Monocular calibration: image=$TOPIC camera=$CAMERA"
  ARGS+=("image:=$TOPIC" "camera:=$CAMERA")
fi

echo "Target: $SIZE inner corners, square=$SQUARE m"
echo "GUI: DISPLAY=${DISPLAY:-:1}; results -> config/calibration/intrinsics/"

"${COMPOSE_DEV[@]}" --profile calib run --rm calib bash -c '
  ros2 run camera_calibration cameracalibrator "$@"
  rc=$?
  out=/ws/config/calibration/intrinsics
  mkdir -p "$out"
  if [ -f /tmp/calibrationdata.tar.gz ]; then
    stamp=$(date +%Y%m%d_%H%M%S)
    cp /tmp/calibrationdata.tar.gz "$out/calibrationdata_${stamp}.tar.gz"
    echo "Saved calibration archive: $out/calibrationdata_${stamp}.tar.gz"
  else
    echo "No /tmp/calibrationdata.tar.gz found; click SAVE in the calibration GUI."
  fi
  exit "$rc"
' bash "${ARGS[@]}"
