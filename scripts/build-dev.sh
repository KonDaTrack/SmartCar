#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
. "${SCRIPT_DIR}/common.sh"

echo "==> docker build --target dev -t ${IMAGE}:dev"
docker build \
  -f "${ROOT}/docker/Dockerfile" \
  --target dev \
  -t "${IMAGE}:dev" \
  "${ROOT}"
