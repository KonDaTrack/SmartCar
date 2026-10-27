# Configuration

- `network.yaml` is the single source of truth for IPs, interfaces, ports and hosts.
- `common.yaml` holds shared ROS defaults.
- `dev.yaml` / `vehicle.yaml` are site overrides.
- `calibration/` holds extrinsics/intrinsics and time offsets.
- `rviz/` holds RViz configs used by the dev `rviz` container.

Containers mount this directory at `/config` (read-only). Site parameters are
never baked into the runtime image. The vehicle release bundle contains a copy
of this directory, so each release records the exact config it ran with.

When an IP or device changes, update `network.yaml` first, then the relevant
override, then commit and redeploy the config. Do not edit files inside a
running container.
