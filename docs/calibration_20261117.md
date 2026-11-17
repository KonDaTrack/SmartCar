# 2026-11-17 Sensor Calibration

Bags processed:

```text
/home/yunle/bags/calib_cam_lidar_20261117_v03
/home/yunle/bags/calib_all_sensors_20261117_v01
```

Dev copies and converted ROS 1 bags:

```text
/home/k/SmartCar/bags/calib_cam_lidar_20261117_v03
/home/k/SmartCar/bags/calib_all_sensors_20261117_v01
/home/k/SmartCar/calibration/bags/*.bag
```

The ROS 2 mcap bags were converted with `rosbags-convert` (`ros2_humble` to
`ros1_noetic`), keeping only standard `sensor_msgs` topics. The `VX1000_*`
custom messages are not needed by the calibration tools.

## Frames and convention

```text
cam   D435i infra1 optical frame
lidar C16 point-cloud frame ("rslidar")
imu   VX1000 raw IMU frame ("imu_link", NED)
```

All matrices `T_a_b` map a point from frame `b` to frame `a`:

```text
p_a = T_a_b * p_b
```

The full result is in `config/calibration/results_20261117.yaml`.

## Results

| Quantity | Value |
|---|---:|
| `td_cam_imu` | `+39.986 ms` |
| `td_lidar_imu` | `-30 ms` (raw `-40 ms`, detrended `-30 ms`) |
| `td_cam_lidar` | `+69.986 ms` |
| camera-LiDAR plane residual | median `8.8 mm`, p90 `34.6 mm` |
| board-centre reprojection | median `21.2 px`, max `46.5 px` |

Camera-LiDAR extrinsic:

```text
T_lidar_cam =
[ 0.999393990 -0.030818437 -0.016182613  0.049744817]
[-0.018148675 -0.064622335 -0.997744747 -0.193433384]
[ 0.029703175  0.997433796 -0.065142487 -0.446228323]
[ 0.000000000  0.000000000  0.000000000  1.000000000]
```

Camera-IMU extrinsic, from the measured camera/IMU mounting:

```text
T_imu_cam =
[0 0 1  1.60]
[1 0 0  0.00]
[0 1 0 -1.07]
[0 0 0  1.00]
```

## Method and caveats

* **Camera-LiDAR**: checkerboard `10 x 7` squares, `0.10 m` squares
  (`9 x 6` inner corners); factory `infra1` `K`, `D=0`. Camera plane from
  `solvePnP`, LiDAR board plane from background-differenced points, then a
  robust point-to-plane least-squares fit over 13 accepted poses.
* **Camera-IMU td**: VINS-Fusion mono+IMU with fixed `body_T_cam0 = T_imu_cam`
  and `estimate_td=1`. Final value was stable at `0.039986 s`.
* **LiDAR-IMU extrinsic**: LI-Init received point clouds and IMU but could not
  accumulate three-axis rotational excitation because the vehicle only yaws;
  roll/pitch progress stayed at 0%. `T_imu_lidar` is therefore derived as:

  ```text
  T_imu_lidar = T_imu_cam * inv(T_lidar_cam)
  ```

  This preserves the requested chain
  `T_imu_cam = T_imu_lidar * T_lidar_cam`, but it is not an independent
  LiDAR-IMU optimization.
* **LiDAR-IMU td**: `tools/estimate_lidar_imu_offset.py` correlation estimate,
  not a full batch optimization. The correlation peak is broad; treat
  `-30 ms` as `+/-10 ms`.
* Before production use, validate with a straight-line and turn LIO run. A
  proper LiDAR-IMU 6-DoF calibration needs a recording with roll and pitch
  excitation (hand-rotate the sensor stack or use a rotating fixture).

## Reproduce

Calibration workspace:

```text
/home/k/SmartCar/calibration/ws
```

Noetic image and scripts:

```bash
docker build -t smartcar-calib:noetic /home/k/SmartCar/calibration
docker run --rm -v /home/k/SmartCar/calibration:/calibration \
  -v /home/k/SmartCar/calibration/ws:/ws \
  -v /home/k/SmartCar/calibration/bags:/bags \
  smartcar-calib:noetic bash /calibration/run_vins.sh \
  /bags/calib_all_sensors_20261117_v01.bag
```

The camera-LiDAR plane solution and validation script are documented in the
session workspace; the result matrices above are the values to consume.
