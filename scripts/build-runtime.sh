#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
. "${SCRIPT_DIR}/common.sh"

VERSION="${1:-${VERSION:-dev}}"
if [ -z "${VERSION}" ]; then
  echo "usage: $0 <version>"
  exit 1
fi

echo "==> docker build --target runtime -t ${IMAGE}:${VERSION}"
docker build \
  -f "${ROOT}/docker/Dockerfile" \
  --target runtime \
  -t "${IMAGE}:${VERSION}" \
  "${ROOT}"
