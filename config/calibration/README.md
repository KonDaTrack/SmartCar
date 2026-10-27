# Calibration

Add calibration files here and record their date/version in the release
manifest.

Planned files:

- `lidar_imu_extrinsics.yaml` — LSLIDAR C16 to VX1000 IMU.
- `lidar_to_base.yaml` — 3D LiDAR mounting on `base_link`.
- `laser_left_to_base.yaml` / `laser_right_to_base.yaml`.
- `time_offsets.yaml` — LiDAR/IMU time offsets.
- Optional camera intrinsics if perception is added later.

Keep large point-cloud/map files out of Git (use the external SSD/NAS).
Small YAML calibration files are small enough to version here.
