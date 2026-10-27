#!/usr/bin/env bash
set -euo pipefail

BASE="${SMARTCAR_DIR:-/opt/smartcar}"
cd "$BASE/current"

echo "== compose ps =="
docker compose -f compose.vehicle.yaml ps

for svc in sensors slam; do
  echo "== $svc topics =="
  docker compose -f compose.vehicle.yaml exec -T "$svc" \
    bash -lc 'timeout 5 ros2 topic list 2>/dev/null | head -20' || true
done

echo "== docker stats =="
docker stats --no-stream --format 'table {{.Name}}\t{{.CPUPerc}}\t{{.MemUsage}}' || true

# TODO: real checks once the drivers exist:
# docker compose exec -T sensors bash -lc 'timeout 5 ros2 topic hz /lslidar_point_cloud'
# docker compose exec -T sensors bash -lc 'timeout 5 ros2 topic hz /scan_left'
# docker compose exec -T slam   bash -lc 'timeout 5 ros2 topic hz /tf'
