#!/usr/bin/env bash
# Build a runtime image and create a self-contained release bundle.
set -euo pipefail

VERSION="${1:-}"
if [ -z "$VERSION" ]; then
  echo "usage: $0 <version>"
  exit 1
fi

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

IMAGE="${IMAGE:-smartcar-ros2}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
VX1000_DEVICE="${VX1000_DEVICE:-/dev/ttyUSB0}"
OUT="$ROOT/releases/$VERSION"

command -v docker >/dev/null || { echo "docker not found"; exit 1; }
command -v zstd >/dev/null || { echo "zstd not found"; exit 1; }
command -v sha256sum >/dev/null || { echo "sha256sum not found"; exit 1; }
git rev-parse --git-dir >/dev/null 2>&1 || { echo "not a git repository"; exit 1; }

if [ -n "$(git status --porcelain)" ]; then
  echo "working tree is dirty; commit/stash before releasing"
  git status --short
  exit 1
fi

echo "==> building $IMAGE:$VERSION"
docker build -f "$ROOT/docker/Dockerfile" --target runtime -t "$IMAGE:$VERSION" "$ROOT"

rm -rf "$OUT"
mkdir -p "$OUT"

echo "==> saving image"
docker save "$IMAGE:$VERSION" | zstd -T0 -19 > "$OUT/image.tar.zst"
( cd "$OUT" && sha256sum image.tar.zst > image.sha256 )

echo "==> assembling release bundle"
cp deploy/compose.vehicle.yaml "$OUT/"
cp deploy/rollback.sh deploy/healthcheck.sh "$OUT/"
cp -a config "$OUT/"
chmod +x "$OUT/rollback.sh" "$OUT/healthcheck.sh"

cat > "$OUT/.env" <<EOF
IMAGE=$IMAGE
IMAGE_TAG=$VERSION
ROS_DOMAIN_ID=$ROS_DOMAIN_ID
SMARTCAR_CONFIG_HOST_DIR=./config
VX1000_DEVICE=$VX1000_DEVICE
EOF

GIT_SHA="$(git rev-parse HEAD)"
GIT_DESC="$(git describe --tags --always --dirty 2>/dev/null || git rev-parse --short HEAD)"
IMAGE_ID="$(docker image inspect -f '{{.Id}}' "$IMAGE:$VERSION")"

cat > "$OUT/manifest.yaml" <<EOF
version: $VERSION
created: $(date -Iseconds)
git_sha: $GIT_SHA
git_describe: $GIT_DESC
image: $IMAGE:$VERSION
image_id: $IMAGE_ID
base_image: ros:humble-ros-base
ros_domain_id: $ROS_DOMAIN_ID
config_dir: ./config
vx1000_device: $VX1000_DEVICE
notes: "Skeleton release; drivers/SLAM still TODO."
EOF

mkdir -p "$ROOT/release-manifests"
cp "$OUT/manifest.yaml" "$ROOT/release-manifests/$VERSION.yaml"

echo "==> release ready: $OUT"
echo "    commit release-manifests/$VERSION.yaml"
echo "    deploy: ./scripts/deploy.sh $VERSION"
