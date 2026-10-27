#!/usr/bin/env bash
set -euo pipefail

cat <<'EOF'
SmartCar scripts (run from anywhere; they cd to the repo root):

  ./scripts/build-dev.sh              Build the dev image
  ./scripts/build-runtime.sh [ver]    Build a runtime image
  ./scripts/dev.sh                    Open the interactive dev container
  ./scripts/up.sh                     Start dev sensors + slam containers
  ./scripts/down.sh                   Stop dev sensors + slam containers
  ./scripts/rviz.sh                   Start RViz (X11 + NVIDIA GPU)
  ./scripts/rviz-stop.sh              Stop RViz
  ./scripts/record.sh                 Record a bag on the dev machine
  ./scripts/replay.sh [bag]           Replay a bag (default: latest)
  ./scripts/release.sh <ver>          Build image + release bundle
  ./scripts/deploy.sh <ver>           rsync + docker load + compose up
  ./scripts/rollback.sh               Switch vehicle to previous release
  ./scripts/health.sh                 Run vehicle healthcheck
  ./scripts/test.sh                   colcon test in the dev container
  ./scripts/clean.sh                  Remove build/ install/ log/

Optional .env (gitignored) is sourced by the scripts if present.
See docs/workflow.md and scripts/README.md.
EOF
