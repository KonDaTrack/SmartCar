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

Camera-LiDAR extrinsic, `T_cam_lidar` (camera <- LiDAR):

```text
T_cam_lidar =
[ 0.999393990 -0.030818437 -0.016182613  0.049744817]
[-0.018148675 -0.064622335 -0.997744747 -0.193433384]
[ 0.029703175  0.997433796 -0.065142487 -0.446228323]
[ 0.000000000  0.000000000  0.000000000  1.000000000]
```

`T_lidar_cam` is its inverse (LiDAR <- camera).

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
  T_imu_lidar = T_imu_cam * T_cam_lidar
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

## Comparison with the 2026-10-30 GLIM run

Local record: `/home/k/SmartCar/slam-bench/full/glim` (bag
`20261030_080720_prepared`). The actual matrix used by GLIM was:

```text
old T_lidar_imu =
[1  0  0 -0.96]
[0 -1  0  0   ]
[0  0 -1 -1.20]
```

Compared with the corrected new `T_lidar_imu`:

- rotation difference: `88.33 deg`, almost exactly yaw
  (`roll 0.93 deg`, `pitch 3.74 deg`)
- translation difference after expressing both in `T_imu_lidar`: `0.21 m`
- old GLIM run loop error: `22.36 m`

The old GLIM log repeatedly reports:

```text
IMU prediction is not good.
Possibly T_lidar_imu is not accurate or IMU bias is not well estimated.
```

So the old value is not a clean ground truth. The main discrepancy is the
C16 point-cloud frame yaw: the raw cloud's forward direction is approximately
`+y`, while the old value assumed `+x`.

### Re-run with the new extrinsic

The same full bag was re-run with the corrected `T_lidar_imu`:

```text
result: /home/k/SmartCar/slam-bench/new_extrinsic_full
config: /home/k/SmartCar/slam-bench/new_extrinsic_full/config_sensors_used.json
compare: /home/k/SmartCar/slam-bench/compare_new_vs_old
```

| Metric | Old | New |
|---|---:|---:|
| loop error | 22.36 m | 20.11 m |
| IMU rotation error | 0.109 deg | 0.086 deg |
| `IMU prediction is not good` warnings | 92 | 96 |
| z range | -14.29 .. 14.56 m | -12.89 .. 2.92 m |

The new extrinsic improves the trajectory closure and IMU rotation error, and
reduces the velocity-prediction ratio, but GLIM still reports poor IMU
prediction. The remaining error is likely dominated by time offset / IMU bias
rather than the LiDAR-IMU orientation alone.

## Camera-bridge validation of LiDAR-IMU

An independent VINS run was made with `estimate_extrinsic=1`, using the
measured camera-IMU mounting as the initial guess. It converged to:

```text
T_imu_cam_vins =
[-0.0390 -0.0397  0.9985  1.5799]
[ 0.9992  0.0017  0.0391 -0.0324]
[-0.0032  0.9992  0.0396 -1.0750]
[ 0       0       0       1     ]
```

Compared with the static mounting:

```text
rotation delta:    3.19 deg
translation delta: 0.039 m
```

Chaining this with the checkerboard `T_cam_lidar`:

```text
T_imu_lidar_vins = T_imu_cam_vins * T_cam_lidar
```

gives a LiDAR-IMU transform within:

```text
rotation delta:    3.19 deg
translation delta: 0.057 m
```

of the current derived `T_imu_lidar`. The old GLIM value is still about
`90.5 deg` away in yaw.

This is a useful independent cross-check of the camera-IMU leg and the
camera-LiDAR leg, but it is not a direct 6-DoF LiDAR-IMU optimization.
Estimating the camera-IMU extrinsic online also shifts `td_cam_imu` from
`39.986 ms` to `45.053 ms`; the two are correlated.

## Static roll/pitch check without vehicle motion

A ground-plane/gravity check was made on a stationary window of the
all-sensor bag (`t=1794906567.532`, gyro norm mean `0.0020 rad/s`):

```text
LiDAR ground-plane normal (LiDAR frame): [ 0.0135,  0.0156,  0.9998]
IMU specific force (IMU frame):          [ 0.0060,  0.0200, -0.9998]
```

After applying the current `T_imu_lidar`, the angle between the two
directions is:

```text
current derived T_imu_lidar: 3.429 deg
old static roll=pi value:     2.085 deg
```

The current yaw/lever-arm result is supported by the camera bridge, but its
roll/pitch is slightly worse than the old static roll=pi assumption. A
minimal roll/pitch correction (3.429 deg about
`[0.3869, -0.92198, -0.01612]`) aligns the ground normal to gravity; the
corrected matrix is stored under `gravity_ground_roll_pitch_check` in the
results YAML. This is only a 2-DoF static check and does not replace a
6-DoF calibration.

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
