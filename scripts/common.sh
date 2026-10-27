#!/usr/bin/env bash
# Shared setup for SmartCar scripts. Source this file; do not execute it directly.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${ROOT}"

# Optional local overrides. .env is gitignored.
if [ -f "${ROOT}/.env" ]; then
  set -a
  # shellcheck disable=SC1091
  . "${ROOT}/.env"
  set +a
fi

IMAGE="${IMAGE:-smartcar-ros2}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
ROS_DISTRO="${ROS_DISTRO:-humble}"
VERSION="${VERSION:-dev}"
VEHICLE="${VEHICLE:-yunle@192.168.100.10}"
VEHICLE_DIR="${VEHICLE_DIR:-/opt/smartcar}"
VX1000_DEVICE="${VX1000_DEVICE:-/dev/ttyUSB0}"

export IMAGE ROS_DOMAIN_ID RMW_IMPLEMENTATION ROS_DISTRO VERSION
export VEHICLE VEHICLE_DIR VX1000_DEVICE

COMPOSE_DEV=(docker compose -f deploy/compose.dev.yaml)
