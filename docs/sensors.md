# Sensors

Inventory as of 2026-10-25. Live UDP data was observed for the LiDARs and the
chassis gateway; model names/roles come from the vehicle launch files and the
last ROS run logs.

| Sensor | Model | Interface | Address / device | ROS topics |
|---|---|---|---|---|
| 3D LiDAR | LSLIDAR C16 (16-line) | Ethernet, UDP 2368/2369 | `192.168.100.201` | `/lslidar_point_cloud` (TF `rslidar`) |
| 2D LiDAR left | RichBeam LakiBeam1 | Ethernet, UDP 2468 | `192.168.100.200` | `/scan_left` (TF `laser_left`) |
| 2D LiDAR right | RichBeam LakiBeam1 | Ethernet, UDP 2568 | `192.168.100.210` | `/scan_right` (TF `laser_right`) |
| GNSS/RTK + IMU | VX1000 | USB serial, 460800 | `/dev/ttyUSB0` (`/dev/gps`) | `/VX1000_ins`, `/VX1000_imu`, `/VX1000_gnss`, `/VX1000_stat`, `/VX1000_cov`, `/VX1000_odom`, `/imu/data`, `/gnss/fix` |
| Chassis | CAN-to-Ethernet gateway | Ethernet, UDP 4001/4002 → `.10:8002` | `192.168.100.110` | `/vehicle_status`, `/odom`, TF `odom -> base_footprint` |
| Camera | Intel RealSense D435i | USB 3 | `8086:0b3a`, serial `947522072480` | **Not used, not recorded** |

## Mounting / TF (from the legacy launch)

- 3D LiDAR: `base_link` + (x=0.96, z=1.4), frame `rslidar`.
- Left 2D: x=1.7, y=+0.66, z=0.15, yaw ≈ +0.795 rad.
- Right 2D: x=1.7, y=-0.66, z=0.15, yaw ≈ -0.795 rad.
- IMU/GPS: `base_link` z=0.2, frames `imu_link` / `gps`.

## Notes

- The VX1000 is an integrated GNSS/RTK + IMU unit; the IMU data used by SLAM
  comes from it. The RealSense built-in IMU is disabled in the legacy launch.
- The chassis provides speed, steering and `total_odometer`. `smartcar_chassis`
  integrates `/vehicle_status` into `/odom` and TF `odom -> base_footprint`
  using the Ackermann model (`wheel_base=1.2`, `is_akm=true`).
- JD01 does not report usable wheel radii in the legacy driver, so
  `left_wheel_speed` / `right_wheel_speed` are zero unless `wheel_radius` is
  set in `config/chassis.yaml`; Ackermann odometry uses speed + steering.
- Steering units are raw CAN 0.1 deg divided by 31.48 (legacy calibration for
  a larger vehicle). Adjust `steer_scale` (chassis) or `steer_angle_scale`
  (odometry) after a straight-line heading test.
- VX1000 `utc_year` is years since 2000 on the wire; the ROS 2 driver adds
  2000. GNSS speed and velocity components use a -100 offset.
- `ins_lat/lon/alt` remain zero and `navi_tick=1` until the INS has a valid
  navigation solution; GNSS-only data (`/gnss/fix`, `/VX1000_gnss`) is still
  available.
- 8 ultrasonic channels exist in the legacy state-machine parameters, but the
  ultrasonic driver is commented out and no data was published. Do not assume
  the hardware is installed.
- No millimeter-wave radar and no IP camera are part of this vehicle.
- Do not run ROS 1 and ROS 2 drivers for the same sensor at the same time:
  LiDAR ports 2368/2369 and 2468/2568, chassis local port 8002.
