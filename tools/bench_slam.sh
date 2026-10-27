#!/usr/bin/env bash
# Placeholder for SLAM benchmarking on a bag.
set -euo pipefail

BAG="${1:-}"
if [ -z "$BAG" ]; then
  echo "usage: $0 /bags/<bag>"
  exit 1
fi

echo "TODO: benchmark SLAM on $BAG"
echo "  - run the SLAM launch with use_sim_time:=true"
echo "  - record CPU/RAM with docker stats / pidstat"
echo "  - record output rate and latency"
echo "  - write results to docs/benchmarks.md"
