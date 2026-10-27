# smartcar_slam

Placeholder package for the SLAM/localization layer.

Inputs available on the vehicle:

- `/lslidar_point_cloud` — LSLIDAR C16 3D LiDAR
- `/VX1000_imu`, `/VX1000_gnss`, `/fused_fix` — VX1000 GNSS/RTK + IMU
- `/vehicle_status` — chassis speed/steering/odometry
- `/tf`, `/tf_static`

The SLAM runs on the vehicle IPC in the `slam` container. The dev machine only
records bags and replays them for offline tuning; it does not run production SLAM.
