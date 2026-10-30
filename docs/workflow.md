# SmartCar workflow

## North star

- One repository, one runtime image, multiple compose profiles.
- Develop and build on the dev machine; the vehicle only runs images.
- Source is mounted in the dev container and baked into the runtime image.
- Site configuration is mounted, never baked into the image.
- GitHub backs up source; images travel over the LAN.
- Production SLAM/localization runs on the vehicle. The vehicle records bags
  locally; the dev machine replays them, builds maps, visualizes with RViz,
  and builds images.

## GitHub

Private repository contains:

- `src/`, `docker/`, `deploy/`, `tools/`, `config/`, `docs/`, `tests/`
- `deps.repos` (pinned driver commits)
- `release-manifests/<version>.yaml`

Never commit:

- `releases/`, `image.tar.zst`
- `bags/`, maps, large point clouds
- `build/`, `install/`, `log/`
- `.env`, SSH keys, Pgyer/WiFi passwords

## Images

| Image | Purpose | Contents |
|---|---|---|
| `smartcar-ros2:dev` | Dev/build/visualize/record | Humble desktop, build tools, RViz, rosbag2 MCAP/zstd |
| `smartcar-ros2:<version>` | Vehicle runtime | Humble base, compiled packages, launch, runtime libs; no GUI/build tools |

The runtime image is built from a clean Git tag. The same image is used for
vehicle containers and for runtime validation on the dev machine.

## Containers

| Machine | Container | Role |
|---|---|---|
| Vehicle | `sensors` | LSLIDAR C16, LakiBeam left/right, VX1000; chassis initially here |
| Vehicle | `slam` | SLAM / localization / map |
| Vehicle | `chassis` (optional) | Split out when the chassis node is stable |
| Vehicle | `vehicle` (optional profile) | Combined debug fallback |
| Vehicle | `record` (profile) | MCAP/zstd recorder + time-offset monitor |
| Dev | `dev` | Interactive build/shell |
| Dev | `rviz` | X11 + NVIDIA GPU visualization |
| Dev | `record` | Headless rosbag2 record |
| Dev | `replay` | One-off bag playback |

No per-driver containers. No ROS 1 bridge. No Foxglove.

## Daily development

```bash
./scripts/build-dev.sh
./scripts/dev.sh
# inside:
cd /ws
colcon build --symlink-install
```

With the dev machine wired to the vehicle switch:

```bash
./scripts/up.sh          # sensors + slam dev containers
./scripts/rviz.sh        # RViz over DDS
./scripts/record.sh      # LiDAR + IMU + chassis + TF, MCAP/zstd
```

Sensors stay up while SLAM is restarted:

```bash
docker compose -f deploy/compose.dev.yaml up -d --no-deps slam
```

Record on the vehicle (the 100M LAN is not used for live recording):

```bash
docker compose -f deploy/compose.vehicle.yaml up -d sensors chassis
docker compose -f deploy/compose.vehicle.yaml --profile record run --rm record
```

See [time_sync.md](time_sync.md) for the calibration and offset workflow.

## Release

```bash
git status                     # must be clean
git tag -a v0.1.0 -m "vehicle v0.1.0"
git push origin main --tags
./scripts/release.sh v0.1.0
```

`tools/release.sh` builds the runtime image and writes:

```text
releases/v0.1.0/
├── image.tar.zst
├── image.sha256
├── compose.vehicle.yaml
├── config/
├── rollback.sh
├── healthcheck.sh
├── manifest.yaml
└── .env
```

The manifest is copied to `release-manifests/v0.1.0.yaml` and committed.

## Deploy and rollback

```bash
./scripts/deploy.sh v0.1.0     # rsync + docker load + compose up
./scripts/health.sh            # compose ps, topics, stats
./scripts/rollback.sh          # switch current -> previous release
```

Vehicle layout:

```text
/opt/smartcar/
├── releases/v0.1.0/
├── releases/v0.1.1/
├── current -> releases/v0.1.1
└── ...
```

## Runtime rules

1. Vehicle never mounts source and never compiles.
2. Launch files are in the image; YAML/calibration are mounted at `/config`.
3. Immutable tags only; no `latest` on the vehicle.
4. Keep the current and previous release for rollback.
5. `network_mode: host` and `ipc: host` for DDS, LiDAR UDP and shared memory.
6. Device access (VX1000 serial) only in the container that needs it.
7. Stop ROS 1 drivers before starting ROS 2 drivers.
8. No camera driver unless something actually uses it.

## Bandwidth

- Sensor input to `.10`: roughly 12 Mbps.
- C16 point cloud to the dev machine: roughly 30–50 Mbps per subscriber.
- On a 100M link, visualize a downsampled `/points_viz` topic; avoid
  full-resolution visualization plus full-resolution recording simultaneously.
- On a 1G link, full-resolution visualization and recording are comfortable.
