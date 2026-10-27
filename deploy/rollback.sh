#!/usr/bin/env bash
set -euo pipefail

BASE="${SMARTCAR_DIR:-/opt/smartcar}"

mapfile -t releases < <(
  find "$BASE/releases" -mindepth 1 -maxdepth 1 -type d -printf '%f\n' | sort -V
)

if [ "${#releases[@]}" -lt 2 ]; then
  echo "need at least two releases to roll back"
  exit 1
fi

current=""
if [ -L "$BASE/current" ]; then
  current="$(basename "$(readlink -f "$BASE/current")")"
fi

prev=""
for r in "${releases[@]}"; do
  if [ "$r" = "$current" ]; then
    break
  fi
  prev="$r"
done

if [ -z "$prev" ]; then
  prev="${releases[${#releases[@]}-2]}"
fi

ln -sfn "$BASE/releases/$prev" "$BASE/current"
cd "$BASE/current"
docker compose -f compose.vehicle.yaml up -d
echo "rolled back: ${current:-unknown} -> $prev"
