# SmartCar

[English](README.md) | [简体中文](README.zh-CN.md)

ROS 2 Humble + Docker workspace for the SmartCar vehicle.

## Status

The vehicle stack, sensor drivers, calibration workflow and recorder are
usable. SLAM integration, a persistent UDP-driver image and a clean
single-pass all-sensor map are still TODO.

### Done

- **Drivers**
  - LSLIDAR C16 3D LiDAR (`/lslidar_point_cloud`, 10 Hz).
  - VX1000 GNSS/RTK + IMU: serial transport and Ethernet UDP transport.
    Publishes `/VX1000_ins|imu|gnss|stat|cov|odom`, `/imu/data`, `/gnss/fix`.
  - JD01 chassis CAN-to-Ethernet gateway and Ackermann `/chassis/odom`.
  - Intel D435i color/depth/infra drivers; camera IMU disabled, VX1000 is the
    only IMU.
- **VX1000 UDP**
  - Same 68-byte frames as the serial link, carried over UDP from
    `192.168.100.100:55325` to vehicle `192.168.100.10:9000`.
  - Verified at 100 packets/s, 330 frames/s, zero checksum errors.
  - Vehicle compose defaults to `SMARTCAR_VX1000_TRANSPORT=udp`; serial remains
    available with `SMARTCAR_VX1000_TRANSPORT=serial`.
- **Recorder reliability**
  - `tools/record.sh` uses a 128 MiB per-topic cache and
    `config/record_qos.yaml` (VX1000 subscription depth 1000).
  - Compression is off by default; the 2.6 s pause at a 20 GB split was
    observed and is documented.
  - `RECORD_TOPICS` accepts single-line and multi-line topic lists.
- **Calibration**
  - Camera-LiDAR checkerboard calibration and camera-IMU VINS time offset.
  - LiDAR-IMU transform derived through the camera-IMU + camera-LiDAR chain.
  - Static ground/gravity roll-pitch check.
  - Results and caveats: `config/calibration/results_20261117.yaml`,
    `docs/calibration_20261117.md`.
- **Recording experiments**
  - LiDAR + RGB + VX1000 and RGBD + stereo + VX1000 bags.
  - Split simultaneous recording: LiDAR + INS in one bag, aligned RGBD in
    another; avoids the camera load affecting the VX1000 recorder.
- **External experiments**
  - ORB-SLAM2 source deployed outside this repository for learning:
    `/home/k/orbslam2_ws` with image `smartcar-orbslam2:noetic`.

### TODO

1. **Persistent vehicle image** — build and deploy a runtime image that
   includes the UDP VX1000 driver, recorder cache/QoS changes and the updated
   compose file. The current vehicle image is still the older release.
2. **VX1000 timestamps** — software-map the VX1000 `gps_week_second` to ROS
   time with a host offset, invalid-GPS fallback and monotonic protection.
   `header.stamp = now()` still bunches samples after a scheduler pause.
3. **Clean map recording** — use `MAX_BAG_SIZE_GB=0` for long all-sensor maps,
   or keep the split-recorder layout. Validate that IMU/INS/ODOM stay at
   100 Hz over the full drive.
4. **SLAM integration** — add launch/config for the chosen LIO/VINS frontend;
   compare candidate extrinsics against RTK GNSS truth.
5. **GNSS truth** — confirm RTK Fixed epochs, antenna lever arm and time
   interpolation before using GNSS as ground truth.
6. **Camera-LiDAR refinement** — the checkerboard result is a rough
   calibration; improve with a better target capture or a mature targetless
   tool.
7. **Chassis steering calibration** — confirm `steer_scale` / wheel base on
   the small vehicle.
8. **Health checks** — implement real topic-rate, TF and SLAM-output checks.
9. **ORB-SLAM2 data path** — convert ROS 2 MCAP bags to ROS 1, run mono
   first, then aligned RGB-D or the infra1/infra2 stereo pair.

## Architecture

- **2 images**
  - `smartcar-ros2:dev` — ROS 2 Humble desktop, build tools, RViz, rosbag2
    MCAP/zstd. Source is bind-mounted; fast iteration.
  - `smartcar-ros2:<version>` — runtime image. Source is compiled into the
    image; no build tools, no GUI.
- **Vehicle: separate containers**
  - `sensors` — C16, LakiBeam left/right, VX1000 (serial or UDP), D435i.
  - `chassis` — CAN gateway receiver and Ackermann odometry.
  - `slam` — SLAM / localization / map.
  - `vehicle` combined profile is kept for debugging.
- **Dev machine: services**
  - `dev` — interactive shell with `src/` and `config/` mounted.
  - `rviz` — X11 + NVIDIA GPU visualization.
  - `record` — headless rosbag2 recorder.
  - `replay` is a one-off container.
- **No ROS 1 bridge in the vehicle stack.** The ORB-SLAM2 ROS 1 workspace is
  an external learning environment, not part of the vehicle runtime.
- **Non-root containers:** images create a `smartcar` user (UID/GID 1000);
  VX1000 serial access uses `dialout`.

## Sensors

| IP / device | Sensor | Notes |
|---|---|---|
| 192.168.100.201 | LSLIDAR C16 3D LiDAR | UDP 2368/2369, `/lslidar_point_cloud`, 10 Hz |
| 192.168.100.200 | LakiBeam1 2D LiDAR, left | UDP 2468, `/scan_left` |
| 192.168.100.210 | LakiBeam1 2D LiDAR, right | UDP 2568, `/scan_right` |
| 192.168.100.110 | Chassis CAN-to-Ethernet | UDP 4001/4002 -> `.10:8002` |
| `/dev/ttyUSB0` (by-id) | VX1000 GNSS/RTK + IMU, serial | 460800 baud, `/dev/gps` |
| 192.168.100.100:55325 | VX1000 Ethernet output | UDP to `.10:9000`, same 68-byte frames |
| USB | Intel RealSense D435i | Color/depth/infra; camera IMU disabled |

The D435i stereo pair is `infra1` + `infra2` (Y8 grayscale); color is a separate
RGB8 sensor. Depth is computed by the IR stereo pair, not by RGB.

See [docs/sensors.md](docs/sensors.md) and [docs/network.md](docs/network.md).

## Recording

Default recorder:

```bash
./scripts/record.sh
```

The recorder writes MCAP to `SMARTCAR_BAGS_HOST_DIR` (default
`/home/yunle/bags`), splits at `MAX_BAG_SIZE_GB`, refuses to start below
`MIN_FREE_GB`, and finalizes metadata on Ctrl-C.

Reliability settings:

```text
MAX_CACHE_SIZE_MB=128        per-topic rosbag2 cache
QOS_PROFILE=/config/record_qos.yaml
COMPRESSION_MODE=none
```

For long maps, set `MAX_BAG_SIZE_GB=0` (or larger than the expected output) to
avoid the ~2.6 s recorder pause at a 20 GB split. If all four camera streams
and INS are needed, prefer two simultaneous recorders:

```text
A: LiDAR + VX1000 + /imu/data + GNSS + chassis
B: RGB or RGBD images + camera_info
```

Both must run at the same time; merge or align by timestamps afterward.

See [docs/time_sync.md](docs/time_sync.md).

## Calibration

Current results:

```text
config/calibration/results_20261117.yaml
config/calibration/time_offsets.yaml
docs/calibration_20261117.md
```

Summary:

- Camera-LiDAR checkerboard: plane residual median 8.8 mm, board-centre
  reprojection median 21.2 px (rough).
- Camera-IMU VINS time offset: `td_cam_imu = 39.986 ms` with fixed extrinsic;
  `45.053 ms` with online extrinsic.
- LiDAR-IMU transform is derived via `T_imu_lidar = T_imu_cam * T_cam_lidar`,
  not an independent 6-DoF optimization. The vehicle cannot roll/pitch, so
  LI-Init could not initialize full excitation.
- Static ground/gravity check constrains only roll/pitch; see the result YAML
  for the optional corrected matrix.
- Use RTK GNSS for independent position truth; INS is a fused GNSS+IMU
  reference, not an independent truth source.

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

On the vehicle, with the dev machine wired to the vehicle LAN:

```bash
./scripts/up.sh      # sensors + slam dev containers
./scripts/rviz.sh    # RViz container (X11 + RTX 5060)
./scripts/record.sh  # rosbag2 MCAP
```

## Release / deploy

```bash
git add . && git commit
./scripts/release.sh v0.1.5     # build runtime image + image.tar.zst + manifest
./scripts/deploy.sh  v0.1.5     # sync over LAN + docker load + compose up
./scripts/health.sh             # run healthcheck on the vehicle
./scripts/rollback.sh           # switch to previous release
```

The vehicle needs Docker, `rsync`, `zstd` and Docker Compose v2. See
[docs/deploy.md](docs/deploy.md).

## Repository map

```text
docker/          Dockerfile, entrypoint, apt package lists
scripts/         build/dev/up/rviz/calib/record/release/deploy shortcuts
src/             ROS 2 packages (bringup, sensors, chassis, slam, msgs, ins)
config/          site configs: network, dev, vehicle, calibration, rviz
deploy/          compose files, deploy/rollback/healthcheck, systemd
tools/           record, release, benchmark scripts
docs/            network, sensors, workflow, calibration, deploy, troubleshooting, ADRs
tests/           unit / bag replay test notes
bags/            local bags (gitignored)
release-manifests/  small release manifests committed to Git
```

## Fixed workflow rules

1. GitHub private repo backs up **source/config/scripts/docs**, never images or
   bags.
2. Dev container mounts `src/`; runtime image bakes source in via multi-stage
   build.
3. The vehicle only runs runtime images and mounts `config/`; no source mount,
   no compiler.
4. Launch files live in the image; site parameters live in `config/`.
5. Immutable image tags on the vehicle; keep current + previous release for
   rollback.
6. No dirty-tree releases.
7. One sensor stack at a time: ROS 1 drivers must not run alongside ROS 2
   drivers on the vehicle.
8. RViz runs on the dev machine; the vehicle runs no GUI.
9. 100M LAN: visualize a downsampled cloud; avoid full-resolution visualization
   and full-resolution bag recording at the same time.
10. Containers run as non-root `smartcar` (UID 1000); do not run the stack as
    root inside the container.

## Drivers

- `smartcar_ins` — VX1000 on `/dev/ttyUSB0` @ 460800, or UDP on port 9000.
  Publishes `/VX1000_ins|imu|gnss|stat|cov|odom`, `/imu/data`, `/gnss/fix`.
- `smartcar_chassis` — receives the `.110` CAN-over-UDP stream on port 8002,
  publishes `/vehicle_status`, and integrates `/chassis/odom` (JD01 Ackermann,
  `wheel_base=1.2`). Its TF is off by default because the SLAM front-end owns
  `/odom` and `odom -> base_link`.
- `smartcar_sensors` / `smartcar_bringup` — C16, LakiBeam and D435i launch
  wiring.
- Site parameters live in `config/chassis.yaml`, `config/lslidar_cx.yaml` and
  `config/network.yaml`.

## External learning workspace

```text
/home/k/orbslam2_ws
  src/ORB_SLAM2        official ORB-SLAM2, OpenCV4-patched
  thirdparty/Pangolin  Pangolin 0.5
  config/D435i_color.yaml
  README.md
```

Docker image: `smartcar-orbslam2:noetic`. ORB-SLAM2 is ROS 1 and does not use
the VX1000 IMU; use it to learn the ORB-SLAM2 pipeline first, then move to a
VIO/LIO stack for the vehicle IMU.
