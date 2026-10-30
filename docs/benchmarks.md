# SLAM benchmark: GLIM vs KISS-SLAM vs Kinematic-ICP

Date: 2026-10-30. All three ran offline in the persistent
`smartcar-slam-bench` container on the development machine (24 cores,
RTX 5060; GLIM and Kinematic were CPU-only). The input is the recorded
vehicle bag `20261030_080720`:

- 637.8 s, C16 at 10 Hz (6,368 scans), VX1000 IMU at 100 Hz,
  chassis odometry at 100 Hz.
- The route includes a climb of roughly 9–14 m; the chassis odometry is
  planar and therefore not an altitude reference.

## Input preparation

The raw bag is unchanged. A derived, uncompressed MCAP was written for the
benchmark because the C16 driver stamps `header.stamp` at the **sweep end**
while `point.time` is 0..0.1 s relative to the **sweep start**, and the bag
has no `/tf` (chassis TF was disabled):

- C16 header moved back by the measured scan duration (median 100.0 ms),
  so `header.stamp` is the scan start and `point.time` remains 0..0.1 s.
- `/tf` synthesized from `/chassis/odom` (`chassis_odom -> base_link`) so
  Kinematic-ICP can use the wheel prior.
- `/imu/data`, `/chassis/odom`, `/tf_static` copied as-is.

The derived bag is 5.2 GB and has 197,697 messages. It is at
`/home/k/SmartCar/slam-bench/bags/20261030_080720_prepared` on the host
(`/results/bags/...` in the container). Preparation script:
`tools/slam_bench/prepare_bag.py`.

## Configurations

| Candidate | Settings |
|---|---|
| GLIM 1.2.2 | CPU configs (`config_odometry_cpu.json`, `config_sub_mapping_cpu.json`, `config_global_mapping_cpu.json`), `T_lidar_imu = [-0.96, 0, -1.2, 1, 0, 0, 0]`, `imu_time_offset = 0`, `points_time_offset = 0`, headless (memory-monitor extension only) |
| KISS-SLAM 0.0.2 | default outdoor config, `kiss_slam_pipeline <bag> --topic /lslidar_point_cloud`; LiDAR only |
| Kinematic-ICP 0.1.1 | `voxel_size = 0.25 m`, `max_range = 50 m`, `max_num_iterations = 20`, `tf_timeout = 0.1 s`, wheel frame `chassis_odom`, base `base_link`; offline TF/spin patch applied |

Notes:

- Kinematic-ICP's default `voxel_size = 1.0` is too coarse for the C16 here;
  it under-estimated the path by roughly 2x (11 m vs 20.7 m over a 45 s
  clip). `0.25 m` recovers the wheel-odometry scale.
- GLIM's `imu_time_offset` was swept over `0`, `+0.05`, `-0.05` s on a
  45 s clip; path length changed by <1%, so the zero offset was used for
  the full run. GLIM still logs an IMU-validation warning (see below).
- KISS-SLAM is LiDAR-only; it does not use the IMU or wheel odometry.

## Results (full bag)

| Metric | GLIM | KISS-SLAM | Kinematic-ICP | Chassis odom (reference) |
|---|---:|---:|---:|---:|
| Poses | 6,337 | 6,368 | 6,358 | 63,773 |
| 2-D path length | 674.3 m | **705.0 m** | 699.9 m | 716.1 m |
| 3-D path length | 685.2 m | 709.6 m | 699.9 m (2-D) | 716.1 m |
| 2-D start/end distance | 18.0 m | **5.0 m** | 245.3 m | 90.3 m |
| Z range | -14.3 .. +14.6 m | -6.3 .. +11.6 m | 0 (2-D) | 0 |
| Wall time (dev) | 110.2 s | **67.6 s** | 291.0 s | — |
| CPU time | 300.8 s | 934.1 s | 290.6 s | — |
| Peak RSS | 2,225 MB | 1,678 MB | 1,003 MB | — |
| Loop closures | 5 non-sequential factors (60 matching factors, 137 submaps) | 0 reported | none | — |
| Map output | 137 submaps, 301 MB dump | 5 local-map PLYs, 12 MB | none | — |

GLIM vs KISS-SLAM in 2-D, after interpolating onto the same time grid:
mean 4.91 m, median 1.59 m, p95 15.5 m, final 14.9 m.

### What the numbers mean

- **KISS-SLAM** is the best self-consistent run on this bag: its 2-D path is
  within 1.6% of the chassis odometry length, and its final position is 5 m
  from the start. It reports no loop closures, so the 5 m figure is residual
  drift, not an optimized loop closure. It is LiDAR-only and therefore
  insensitive to the IMU time offset and extrinsic errors.
- **GLIM** is stable enough to finish and produces a globally optimized
  trajectory, but is about 6% shorter than the wheel path and ends 18 m from
  the start. It has occasional large steps (max 0.59 m, peak speed
  5.9 m/s versus a 1.5 m/s vehicle maximum), and logs
  “IMU prediction is not good / T_lidar_imu may not be accurate”. The IMU
  extrinsic, time offset, and IMU noise need another tuning pass before it
  is trusted as the live estimator.
- **Kinematic-ICP** matches the chassis odometry for the first ~40 s
  (ratio 0.98), then diverges; by 120 s the 2-D position is 38 m away from
  the wheel path, and the final position is 245 m from the start. The
  route's climb is outside its planar motion model, and the chassis
  odometry itself is questionable (see below). It is not suitable for this
  site as-is.

### Chassis odometry caveat

The chassis odometry and both LiDAR trajectories disagree in the sign of the
lateral axis: KISS-SLAM and GLIM both turn towards +y, while `/chassis/odom`
turns towards -y. The wheel odometry also ends 90 m from the start after a
716 m run. Before using wheel odometry as a prior (Kinematic-ICP) or for
fusion, verify:

1. the C16 point-cloud y-axis convention and the `base_link -> rslidar`
   static transform;
2. the steering-angle sign and `steer_scale` in `config/chassis.yaml`.

This is separate from the SLAM comparison; KISS-SLAM and GLIM do not consume
the wheel odometry.

## Recommendation

- **Offline map of this bag:** KISS-SLAM. Best trajectory consistency,
  smallest loop error, usable local maps, and no dependence on the IMU/wheel
  calibration. It is a Python offline pipeline, not a live ROS 2 node.
- **Live ROS 2:** GLIM is the only one of the three that is a native ROS 2
  node. Use it only after fixing the IMU-validation warning and checking the
  large-step/loop behavior. If live operation can tolerate a LiDAR-only
  front end, use the KISS-SLAM ROS 2 wrapper (or KISS-ICP + EKF) instead.
- **Kinematic-ICP:** keep only for flat sites and only after the chassis
  odometry sign/scale is verified. It needs the patched offline TF handling
  and a C16-appropriate voxel size.

## Reproduce

```bash
# one-time, inside a persistent smartcar-ros2:dev container
docker exec -u root smartcar-slam-bench bash /ws/tools/slam_bench/setup.sh

# derive the benchmark bag from the raw recording
docker exec smartcar-slam-bench python3 /ws/tools/slam_bench/prepare_bag.py \
  /bags-bench/20261030_080720 /results/bags/20261030_080720_prepared

# GLIM (stops itself after the CPU goes idle and writes /tmp/dump)
docker exec smartcar-slam-bench python3 /ws/tools/slam_bench/run_timed.py \
  --out /results/full/glim/time.json --idle-stop --min-runtime 20 \
  --idle-window 10 --max-runtime 600 -- \
  bash /ws/tools/slam_bench/env.sh ros2 run glim_ros glim_rosbag \
  /results/bags/20261030_080720_prepared

# KISS-SLAM (writes ./slam_output/<timestamp>)
docker exec -w /results/full/kiss smartcar-slam-bench \
  kiss_slam_pipeline /results/bags/20261030_080720_prepared \
  --topic /lslidar_point_cloud

# Kinematic-ICP
docker exec smartcar-slam-bench python3 /ws/tools/slam_bench/run_timed.py \
  --out /results/full/kinematic/time.json --max-runtime 900 -- \
  bash /ws/tools/slam_bench/env.sh ros2 run kinematic_icp \
  kinematic_icp_offline_node --ros-args \
  -p lidar_topic:=/lslidar_point_cloud \
  -p bag_filename:=/results/bags/20261030_080720_prepared \
  -p output_dir:=/results/full/kinematic \
  -p use_2d_lidar:=false -p lidar_odom_frame:=odom_lidar \
  -p wheel_odom_frame:=chassis_odom -p base_frame:=base_link \
  -p publish_odom_tf:=false -p invert_odom_tf:=true \
  -p tf_timeout:=0.1 -p voxel_size:=0.25 -p max_range:=50.0 \
  -p max_num_iterations:=20
```

Raw results and maps are under `/home/k/SmartCar/slam-bench/full/` on the
development machine. The benchmark container is disposable: the setup script
is the only persistent state, and the patched Kinematic source is kept in
`tools/slam_bench/patches/`.
