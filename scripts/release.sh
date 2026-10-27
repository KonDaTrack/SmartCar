#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
. "${SCRIPT_DIR}/common.sh"

VERSION="${1:-}"
if [ -z "${VERSION}" ]; then
  echo "usage: $0 <version>"
  exit 1
fi

exec "${ROOT}/tools/release.sh" "${VERSION}"
