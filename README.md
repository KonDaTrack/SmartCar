# SmartCar

ROS 2 Humble + Docker workspace for the SmartCar vehicle.

**Status: skeleton.** The image / container / release / deploy workflow is in place. Actual sensor, chassis and SLAM nodes are still TODO: the launch files currently start a `keepalive` placeholder so containers and the workflow can be tested end to end.

## Architecture

- **2 images**
  - `smartcar-ros2:dev` — ROS 2 Humble desktop, build tools, RViz, rosbag2 MCAP/zstd. Source is bind-mounted; fast iteration.
  - `smartcar-ros2:<version>` — runtime image. Source is compiled into the image; no build tools, no GUI.
- **Vehicle: 2 containers (start state)**
  - `sensors` — LSLIDAR C16, LakiBeam left/right, VX1000. Chassis starts here too.
  - `slam` — SLAM / localization / map.
  - `chassis` can be split out later; `vehicle` combined profile is kept for debugging.
- **Dev machine: 3 services**
  - `dev` — interactive shell with `src/` and `config/` mounted.
  - `rviz` — X11 + NVIDIA GPU visualization.
  - `record` — headless `ros2 bag record` to the dev machine.
  - `replay` is a one-off container.
- **No ROS 1 bridge, no Foxglove, no camera recording.**

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

## Not implemented yet

- LSLIDAR / LakiBeam / VX1000 driver wiring.
- `smartcar_chassis` ROS 2 node for the `.110` CAN-Ethernet gateway.
- SLAM launch and parameters.
- Calibration files and RViz configuration.
- Real health checks (topic rate, TF, SLAM output).

See [docs/workflow.md](docs/workflow.md) for the full workflow and [docs/adr/](docs/adr/) for decisions.
