#!/usr/bin/env bash
# Start the D435i driver locally for intrinsic calibration.
#
# Use this when the camera is plugged directly into the development host.
# The camera container is headless; calibration GUIs run in `calib` or `rqt`.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
. "${SCRIPT_DIR}/common.sh"

if ! lsusb 2>/dev/null | grep -qiE '8086:0b3a|Intel.*RealSense'; then
  echo "warning: no Intel RealSense found by lsusb" >&2
  echo "         check the USB 3 cable/port and 'lsusb -t' for 5000M" >&2
fi

"${COMPOSE_DEV[@]}" --profile calib up -d calib-camera
echo "calib-camera started"
echo "Stop with: docker compose -f deploy/compose.dev.yaml --profile calib stop calib-camera"
