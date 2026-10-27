# Changelog

## Unreleased

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
