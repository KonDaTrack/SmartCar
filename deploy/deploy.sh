#!/usr/bin/env bash
set -euo pipefail

VERSION="${1:-}"
if [ -z "$VERSION" ]; then
  echo "usage: $0 <version>"
  exit 1
fi

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOCAL_RELEASE="$ROOT/releases/$VERSION"
VEHICLE="${VEHICLE:-yunle@192.168.100.10}"
REMOTE_DIR="${VEHICLE_DIR:-/opt/smartcar}"

if [ ! -d "$LOCAL_RELEASE" ]; then
  echo "missing release: $LOCAL_RELEASE"
  echo "run: tools/release.sh $VERSION"
  exit 1
fi

command -v rsync >/dev/null || { echo "rsync not found on dev machine"; exit 1; }
command -v ssh >/dev/null || { echo "ssh not found"; exit 1; }

echo "==> checking vehicle prerequisites"
ssh "$VEHICLE" 'command -v docker >/dev/null && command -v rsync >/dev/null && command -v zstd >/dev/null' \
  || { echo "vehicle needs docker, rsync and zstd"; exit 2; }
ssh "$VEHICLE" 'docker compose version >/dev/null 2>&1' \
  || { echo "vehicle needs Docker Compose v2 (docker compose)"; exit 2; }

echo "==> syncing $VERSION to $VEHICLE:$REMOTE_DIR/releases/$VERSION"
ssh "$VEHICLE" "mkdir -p '$REMOTE_DIR/releases/$VERSION'"
rsync -avP "$LOCAL_RELEASE/" "$VEHICLE:$REMOTE_DIR/releases/$VERSION/"

echo "==> loading image and starting vehicle stack"
ssh "$VEHICLE" "REMOTE_DIR='$REMOTE_DIR' VERSION='$VERSION' bash -s" <<'EOS'
set -euo pipefail
cd "$REMOTE_DIR/releases/$VERSION"
sha256sum -c image.sha256
zstd -d -f image.tar.zst -o image.tar
docker load -i image.tar
rm -f image.tar
ln -sfn "$REMOTE_DIR/releases/$VERSION" "$REMOTE_DIR/current"
cd "$REMOTE_DIR/current"
docker compose -f compose.vehicle.yaml up -d
docker image prune -f >/dev/null || true
EOS

echo "==> deployed $VERSION"
echo "    next: make health"
