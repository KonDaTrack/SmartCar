# SmartCar

ROS 2 Humble + Docker workspace for the SmartCar vehicle.

**Status: drivers in progress.** The image / container / release / deploy
workflow is in place. LiDAR wiring, the VX1000 INS/IMU/GNSS serial driver and
the JD01 chassis gateway + wheel odometry are implemented and were verified on
the vehicle. SLAM, calibration and health checks remain TODO.

## Architecture

- **2 images**
  - `smartcar-ros2:dev` — ROS 2 Humble desktop, build tools, RViz, rosbag2 MCAP/zstd. Source is bind-mounted; fast iteration.
  - `smartcar-ros2:<version>` — runtime image. Source is compiled into the image; no build tools, no GUI.
- **Vehicle: separate containers**
  - `sensors` — LSLIDAR C16, LakiBeam left/right, VX1000 (`SMARTCAR_START_INS=1`).
  - `chassis` — CAN gateway receiver and odometry (`SMARTCAR_START_CHASSIS=1`).
  - `slam` — SLAM / localization / map.
  - `vehicle` combined profile is kept for debugging.
- **Dev machine: 3 services**
  - `dev` — interactive shell with `src/` and `config/` mounted.
  - `rviz` — X11 + NVIDIA GPU visualization.
  - `record` — headless `ros2 bag record` to the dev machine.
  - `replay` is a one-off container.
- **No ROS 1 bridge, no Foxglove, no camera recording.**
- **Non-root containers:** images create a `smartcar` user (UID/GID 1000); dev
  build artifacts stay owned by the host user, and VX1000 serial access uses
  `dialout`.

## Sensors (current inventory)

| IP / device | Sensor | Notes |
|---|---|---|
| 192.168.100.201 | LSLIDAR C16 3D LiDAR | UDP 2368/2369, `/lslidar_point_cloud` |
| 192.168.100.200 | LakiBeam1 2D LiDAR, left | UDP 2468, `/scan_left` |
| 192.168.100.210 | LakiBeam1 2D LiDAR, right | UDP 2568, `/scan_right` |
| 192.168.100.110 | Chassis CAN-to-Ethernet | UDP 4001/4002 -> `.10:8002` |
| `/dev/ttyUSB0` (by-id) | VX1000 GNSS/RTK + IMU | 460800 baud, `/dev/gps` |
| USB | Intel RealSense D435i | **Not used / not recorded** |

See [docs/sensors.md](docs/sensors.md) and [docs/network.md](docs/network.md).

## Quick start

```bash
cp .env.example .env

# 1. Build the development image
./scripts/build-dev.sh

# 2. Open the dev container
./scripts/dev.sh
# inside:
cd /ws
colcon build --symlink-install
source install/setup.bash
```

Live sensor work: connect the dev machine's `enp5s0` to the vehicle switch, then:

```bash
./scripts/up.sh      # sensors + slam dev containers
./scripts/rviz.sh    # RViz container (X11 + RTX 5060)
./scripts/record.sh  # rosbag2 MCAP + zstd, written to bags/
```

## Release / deploy

```bash
git add . && git commit
./scripts/release.sh v0.1.0     # build runtime image + image.tar.zst + manifest
./scripts/deploy.sh  v0.1.0     # rsync over LAN + docker load + compose up
./scripts/health.sh             # run healthcheck on the vehicle
./scripts/rollback.sh           # switch to previous release
```

The vehicle only needs Docker, `rsync`, `zstd` and the Docker Compose v2 plugin. See [docs/deploy.md](docs/deploy.md).

## Repository map

```text
docker/          Dockerfile, entrypoint, apt package lists
scripts/         build/dev/up/rviz/record/release/deploy shortcuts
src/             ROS 2 packages (bringup, sensors, chassis, slam, msgs)
config/          site configs: network, dev, vehicle, calibration, rviz
deploy/          compose files, deploy/rollback/healthcheck, systemd
tools/           record, release, benchmark scripts
docs/            network, sensors, workflow, deploy, troubleshooting, ADRs
tests/           unit / bag replay test notes
bags/            local bags (gitignored)
release-manifests/  small release manifests committed to Git
```

## Fixed workflow rules

1. GitHub private repo backs up **source/config/scripts/docs**, never images or bags.
2. Dev container mounts `src/`; runtime image bakes source in via multi-stage build.
3. The vehicle only runs runtime images and mounts `config/`; no source mount, no compiler.
4. Launch files live in the image; site parameters live in `config/`.
5. Immutable image tags on the vehicle; keep current + previous release for rollback.
6. No dirty-tree releases.
7. One sensor stack at a time: ROS 1 drivers must not run alongside ROS 2 drivers.
8. RViz runs on the dev machine; the vehicle runs no GUI.
9. 100M LAN: visualize a downsampled cloud; avoid full-resolution visualization + full-resolution bag recording at the same time.
10. Containers run as non-root `smartcar` (UID 1000); do not run the stack as root inside the container.

## Drivers

- `smartcar_ins` — VX1000 on `/dev/ttyUSB0` @ 460800. Publishes
  `/VX1000_ins|imu|gnss|stat|cov|odom` plus standard `/imu/data` and
  `/gnss/fix`.
- `smartcar_chassis` — receives the `.110` CAN-over-UDP stream on port 8002,
  publishes `/vehicle_status`, and integrates it into `/chassis/odom`
  (JD01 Ackermann, `wheel_base=1.2`). Its TF is off by default because the
  SLAM front-end owns `/odom` and `odom -> base_link`.
- Site parameters live in `config/chassis.yaml`. The axle geometry and the
  `steer_scale` / `steer_angle_scale` heading calibration must be checked on
  the real vehicle.
- LIO recording is done on the vehicle (`record` compose profile), not over the
  dev LAN. Default topics: C16, VX1000 IMU, chassis status/odometry and TF.
  A time-offset monitor runs alongside the recorder; see
  [docs/time_sync.md](docs/time_sync.md).
- Offline regression (synthetic serial + UDP frames):

  ```bash
  docker compose -f deploy/compose.dev.yaml run --rm \
    -e ROS_DOMAIN_ID=77 dev python3 /ws/tools/smoke_test.py
  ```

## Not implemented yet

- INS navigation initialization / RTK configuration; `ins_lat/lon` stay zero
  while `navi_tick=1`.
- Chassis steering calibration for the small vehicle (legacy `/31.48` divisor).
- SLAM launch and parameters. The offline GLIM / KISS-SLAM / Kinematic-ICP
  comparison is in [docs/benchmarks.md](docs/benchmarks.md).
- Calibration files and RViz configuration.
- Real health checks (topic rate, TF, SLAM output).

See [docs/workflow.md](docs/workflow.md) for the full workflow and [docs/adr/](docs/adr/) for decisions.
