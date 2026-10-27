# Sensors

Inventory as of 2026-10-25. Live UDP data was observed for the LiDARs and the
chassis gateway; model names/roles come from the vehicle launch files and the
last ROS run logs.

| Sensor | Model | Interface | Address / device | ROS topics |
|---|---|---|---|---|
| 3D LiDAR | LSLIDAR C16 (16-line) | Ethernet, UDP 2368/2369 | `192.168.100.201` | `/lslidar_point_cloud` (TF `rslidar`) |
| 2D LiDAR left | RichBeam LakiBeam1 | Ethernet, UDP 2468 | `192.168.100.200` | `/scan_left` (TF `laser_left`) |
| 2D LiDAR right | RichBeam LakiBeam1 | Ethernet, UDP 2568 | `192.168.100.210` | `/scan_right` (TF `laser_right`) |
| GNSS/RTK + IMU | VX1000 | USB serial, 460800 | `/dev/ttyUSB0` (`/dev/gps`) | `/VX1000_imu`, `/VX1000_gnss`, `/VX1000_ins`, `/fused_fix` |
| Chassis | CAN-to-Ethernet gateway | Ethernet, UDP 4001/4002 → `.10:8002` | `192.168.100.110` | `/vehicle_status` (speed, steer, wheel speeds, total odometer) |
| Camera | Intel RealSense D435i | USB 3 | `8086:0b3a`, serial `947522072480` | **Not used, not recorded** |

## Mounting / TF (from the legacy launch)

- 3D LiDAR: `base_link` + (x=0.96, z=1.4), frame `rslidar`.
- Left 2D: x=1.7, y=+0.66, z=0.15, yaw ≈ +0.795 rad.
- Right 2D: x=1.7, y=-0.66, z=0.15, yaw ≈ -0.795 rad.
- IMU/GPS: `base_link` z=0.2, frames `imu_link` / `gps`.

## Notes

- The VX1000 is an integrated GNSS/RTK + IMU unit; the IMU data used by SLAM
  comes from it. The RealSense built-in IMU is disabled in the legacy launch.
- The chassis provides speed, steering, wheel speeds and `total_odometer`; it
  does not provide a full `nav_msgs/Odometry` pose. A ROS 2 odometry node would
  need to integrate `/vehicle_status` if wheel odometry is required.
- 8 ultrasonic channels exist in the legacy state-machine parameters, but the
  ultrasonic driver is commented out and no data was published. Do not assume
  the hardware is installed.
- No millimeter-wave radar and no IP camera are part of this vehicle.
- Do not run ROS 1 and ROS 2 drivers for the same sensor at the same time:
  LiDAR ports 2368/2369 and 2468/2568, chassis local port 8002.
