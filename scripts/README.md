# Scripts

Makefile removed. These are plain Bash scripts; each one prints the underlying
Docker/Compose command where useful.

| Script | Purpose |
|---|---|
| `build-dev.sh` | Build `smartcar-ros2:dev` from `docker/Dockerfile` |
| `build-runtime.sh <ver>` | Build `smartcar-ros2:<ver>` runtime image |
| `dev.sh` | `docker compose run --rm dev` |
| `up.sh` / `down.sh` | Start/stop dev `sensors` + `slam` containers |
| `rviz.sh` / `rviz-stop.sh` | Start/stop the dev RViz container |
| `record.sh` | Record selected topics to `bags/` with MCAP + zstd |
| `replay.sh [bag]` | Replay a bag with `--clock` |
| `release.sh <ver>` | Wrapper for `tools/release.sh` |
| `deploy.sh <ver>` | Wrapper for `deploy/deploy.sh` |
| `rollback.sh` / `health.sh` | SSH to the vehicle and run the release scripts |
| `test.sh` | `colcon test` inside the dev container |
| `clean.sh` | Remove `build/`, `install/`, `log/` |
| `help.sh` | Print this list |

Notes:

- Scripts source `scripts/common.sh`, which `cd`s to the repo root and loads
  `.env` if present (`.env` is gitignored).
- The build scripts explicitly use `-f docker/Dockerfile`; Docker's default
  `./Dockerfile` lookup does not apply because the Dockerfile lives in `docker/`.
- Complex multi-step logic stays in `tools/release.sh`, `deploy/deploy.sh`,
  `deploy/rollback.sh` and `deploy/healthcheck.sh`.
