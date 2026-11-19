# SLAM benchmark harness

Offline comparison of GLIM, KISS-SLAM and Kinematic-ICP on a recorded
SmartCar bag. Everything runs in a **persistent `smartcar-ros2:dev`
container**; no extra image is built.

Results are in [docs/benchmarks.md](../../docs/benchmarks.md).

## Container

```bash
docker run -d --name smartcar-slam-bench --network host --ipc host \
  -v "$PWD":/ws -v "$PWD/config":/config:ro \
  -v /home/k/SmartCar/bags:/bags-bench:ro \
  -v /home/k/SmartCar/slam-bench:/results \
  smartcar-ros2:dev sleep infinity

docker exec -u root smartcar-slam-bench bash /ws/tools/slam_bench/setup.sh
```

`setup.sh` installs GLIM from the koide3 apt repository, builds the patched
Kinematic-ICP ROS 2 wrapper, and pip-installs KISS-SLAM, rosbags, evo and
matplotlib. The container's writable layer holds them; deleting the container
only loses the installation, not the scripts.

## Scripts

| Script | Purpose |
|---|---|
| `setup.sh` | Install GLIM, Kinematic-ICP and KISS-SLAM in the container |
| `env.sh` | Source ROS + the Kinematic workspace, then exec a command |
| `prepare_bag.py` | Move C16 headers to scan start, synthesize `/tf` from `/chassis/odom`, write an uncompressed MCAP |
| `bag_summary.py` | Topic counts, C16 time convention, wheel path |
| `make_clip.sh` | Short clip from a bag (uncompressed by default; GLIM cannot read `.mcap.zstd`) |
| `run_timed.py` | Wall/CPU/max-RSS measurement; `--idle-stop` sends SIGINT to GLIM after processing |
| `trajectory_metrics.py` | Start-normalized path/loop metrics and XY plot |
| `compare_to_odom.py` | Compare a TUM trajectory with `/chassis/odom` over time |
| `odom_to_tum.py` | Export `/chassis/odom` to TUM |
| `export_maps.py` | Merge KISS PLYs / GLIM submaps and write PCD maps |
| `kiss_loop_loose.yaml` | KISS-SLAM diagnostic config for this site's loop gates |
| `glim_offset_sweep.sh` | Sweep GLIM `imu_time_offset` on a clip |
| `kinematic_variants.sh` | Kinematic deskew/regularization variants |
| `build_kinematic.sh` | Rebuild the vendored/patch Kinematic-ICP wrapper |

`patches/kinematic_icp_ros_bench.patch` contains the benchmark-only changes
to Kinematic-ICP: spin the node so bag TF reaches its tf2 buffer, materialize
TF from `/chassis/odom`, handle C16 end-stamped clouds, and wait for TF.

## Typical flow

```bash
# derive a clean bag once
docker exec smartcar-slam-bench python3 /ws/tools/slam_bench/prepare_bag.py \
  /bags-bench/20261030_080720 /results/bags/20261030_080720_prepared

# run the three candidates (see docs/benchmarks.md for full commands)
# GLIM
docker exec smartcar-slam-bench python3 /ws/tools/slam_bench/run_timed.py \
  --out /results/full/glim/time.json --idle-stop --min-runtime 20 \
  --idle-window 10 --max-runtime 600 -- \
  bash /ws/tools/slam_bench/env.sh ros2 run glim_ros glim_rosbag \
  /results/bags/20261030_080720_prepared

# KISS-SLAM
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

GLIM does not exit when its bag reader reaches the end; `--idle-stop` watches
CPU and sends SIGINT, which triggers `/tmp/dump` (trajectories, submaps,
graph). KISS-SLAM writes `./slam_output/<timestamp>` relative to the working
directory. Kinematic-ICP writes a TUM file into `output_dir`.

## Caveats

- The bag route has elevation; the chassis odometry is planar. Do not use its
  z/loop error as ground truth.
- The chassis odometry and LiDAR trajectories disagree in lateral sign on
  this recording; verify the C16 y-axis and steering sign before using wheel
  odometry as a prior.
- KISS-SLAM is an offline Python pipeline, not a live ROS 2 node.
- Kinematic-ICP assumes planar motion and is not suitable for this route
  without further work.
