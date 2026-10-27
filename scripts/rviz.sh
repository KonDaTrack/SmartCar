#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
. "${SCRIPT_DIR}/common.sh"

HOST_USER="$(id -un)"
xhost +si:localuser:"${HOST_USER}" >/dev/null 2>&1 || true
"${COMPOSE_DEV[@]}" --profile viz up -d rviz
echo "RViz started. Stop with ./scripts/rviz-stop.sh"
