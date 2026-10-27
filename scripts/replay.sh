#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
. "${SCRIPT_DIR}/common.sh"

BAG="${1:-${BAG:-latest}}"
export BAG

exec "${COMPOSE_DEV[@]}" --profile replay run --rm replay
