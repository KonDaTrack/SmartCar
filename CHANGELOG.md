# Changelog

## v0.1.0-skeleton - 2026-10-25

- Initial SmartCar repository skeleton.
- Multi-stage Dockerfile with `dev` and `runtime` targets.
- Compose files for dev (`dev`, `sensors`, `slam`, `rviz`, `record`, `replay`) and vehicle (`sensors`, `slam`, optional `chassis`, combined `vehicle`).
- Placeholder ROS 2 bringup package with `keepalive` node and launch files.
- Placeholder `smartcar_sensors`, `smartcar_chassis`, `smartcar_slam`, `smartcar_msgs` packages.
- Network/sensor/workflow/deploy documentation.
- Release manifest, LAN deploy, rollback and healthcheck scripts.
