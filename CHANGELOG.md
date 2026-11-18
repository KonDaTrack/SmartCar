# Changelog

## Unreleased

- Add a UDP transport to the VX1000 driver. The Ethernet output carries the
  same 68-byte frames as the serial link (`192.168.100.100:55325` to
  `192.168.100.10:9000`); the vehicle compose defaults to UDP and can fall
  back with `SMARTCAR_VX1000_TRANSPORT=serial`.
- Raise the rosbag2 recorder cache to 128 MiB and add
  `config/record_qos.yaml` (VX1000 subscription depth 1000). A 60 s
  all-sensor test kept INS/IMU/ODOM at 100 Hz; the previous defaults dropped
  INS/ODOM to 60-70 Hz while camera images were written.
- Raise the VX1000 publisher history depth from 100 to 1000 frames (~10 s)
  so a short recorder pause does not let the publisher discard the oldest
  IMU/INS/ODOM frames.
- Accept multi-line `RECORD_TOPICS` in `tools/record.sh`; plain `read -a`
  only used the first line and silently recorded one topic.
- Install the Intel RealSense camera driver and image-transport compression
  plugins in the runtime image.
- Publish a provisional `base_link -> camera_link` TF for the D435i; the
  value still needs to be measured on the vehicle.
- Document which sensor timestamps come from the host clock versus a sensor
  clock, and when soft-sync offsets remain valid across restarts.
- Wire the LSLIDAR C16 (`192.168.100.201`) and LakiBeam left/right
  (`192.168.100.200/.210`) drivers into `sensors.launch.py`.
- Add `config/lslidar_cx.yaml` with the site IP, frame and topic settings.
- Add `SMARTCAR_HOST_IP` to the dev/vehicle compose environments.
- Run containers as a non-root `smartcar` user (UID/GID 1000) instead of root.
  The ROS base images have no `ubuntu` user; the user is created in the image.
- Added `dialout`/`video`/`render` group handling for the VX1000 serial device
  and RViz GPU access.
- Fixed `smartcar_msgs/package.xml` to use `rosidl_interface_packages`, so
  message generation builds in the runtime image.
- Replaced the Makefile with plain `scripts/*.sh` entry points.
- Build/release commands now pass `-f docker/Dockerfile`; the Dockerfile lives
  under `docker/`, not at the repository root.
- Added `scripts/README.md` and `scripts/help.sh`.

## v0.1.0-skeleton - 2026-10-25

- Initial SmartCar repository skeleton.
- Multi-stage Dockerfile with `dev` and `runtime` targets.
- Compose files for dev (`dev`, `sensors`, `slam`, `rviz`, `record`, `replay`) and vehicle (`sensors`, `slam`, optional `chassis`, combined `vehicle`).
- Placeholder ROS 2 bringup package with `keepalive` node and launch files.
- Placeholder `smartcar_sensors`, `smartcar_chassis`, `smartcar_slam`, `smartcar_msgs` packages.
- Network/sensor/workflow/deploy documentation.
- Release manifest, LAN deploy, rollback and healthcheck scripts.
