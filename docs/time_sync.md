# Time sync and vehicle recording

Scope: LIO sensors — **LSLIDAR C16, VX1000 IMU and the JD01 chassis odometry**.
The 2D LiDARs are not recorded. The RealSense camera is optional and is
recorded only when `RECORD_CAMERA=1`.

## Current time bases

| Topic | Stamp source | Notes |
|---|---|---|
| `/lslidar_point_cloud` | host `now()` at packet receive; cloud header = sweep end | `use_time_service: false`; per-point `time` field is present (`pcl_type: false`) |
| `/imu/data` | host `now()` after serial parse | VX1000 over FTDI at 460800 |
| `/vehicle_status` | host `now()` on UDP receive | `.110` gateway, 100 Hz |
| `/chassis/odom` | copies `/vehicle_status` stamp | Ackermann integration; separate from SLAM `/odom` |
| `/camera/*` (when enabled) | RealSense driver frame timestamp | Not started/recorded by default; global/hardware timestamp domain, **not** assumed to be host `now()` |

C16, VX1000 and chassis all share the **host clock**, so there is no relative
clock drift between them. What remains is transport/processing latency and
jitter. This is what the soft-sync tools measure; it is not a replacement for
PPS/PTP.

### What the soft-sync offsets actually are

`wheel_to_imu_offset_ms` and `lidar_to_imu_offset_ms` are **constant latency
estimates**, not clock offsets. With `use_time_service: false`, the C16 driver
stamps each packet with `this->get_clock()->now()` and derives the cloud stamp
from the sweep end; the VX1000 driver stamps every message with `now()` as
well, and the chassis gateway stamps `/vehicle_status` with `now()`. Therefore
the relative offsets do not change just because the machine rebooted.

They can change when the pipeline changes: USB latency timer, hub/port, CPU
load, network path, driver version, or switching a sensor to its own clock.
They do **not** remain valid if one side starts using sensor time. Examples:

- setting C16 `use_time_service: true` (GPS/PTP/NTP mode);
- making the VX1000 driver stamp with GPS week/second instead of `now()`;
- recording the RealSense camera, whose driver timestamps frames in the
  camera's global/hardware domain.

In those cases the offset is session-dependent unless the sensor clock is
hardware-locked to UTC (PPS + GPRMC/PTP) and the host is also locked. Measure
the camera-to-host offset on the vehicle before mixing it with LiDAR/IMU.

## Static TF / extrinsics

`sensors.launch.py` publishes the current initial extrinsics:

```text
base_link -> rslidar    x=1.2  y=0  z=1.47  rpy=0
base_link -> imu_link    x=0  y=0  z=0  roll=pi, pitch=0, yaw=0
base_link -> vx1000_link x=0  y=0  z=0  roll=pi, pitch=0, yaw=0
base_link -> gps         x=0  y=0  z=0  roll=pi, pitch=0, yaw=0
base_link -> camera_link x=1.6  y=0  z=1.27  rpy=0 (x/z measured; y/rpy TBD)
```

The values are recorded in
[`config/calibration/lidar_imu_extrinsics.yaml`](../config/calibration/lidar_imu_extrinsics.yaml).
Verify before recording:

```bash
ros2 run tf2_ros tf2_echo base_link rslidar
ros2 run tf2_ros tf2_echo base_link imu_link
ros2 topic echo /tf_static --once
```

The VX1000 installation translation is configured inside VxView. All
VX1000-owned frames (`vx1000_link`, `gps`, raw `imu_link`) use the same vendor
NED body axes (x-forward, y-right, z-down; at rest `acc_z ~= -9.8`). Each
`base_link -> ...` TF therefore has a 180 deg roll so TF converts them to ROS
REP-103 (x-forward, y-left, z-up, rest `acc_z ~= +9.8`). Do not remap the raw
IMU values in the driver. Do not hide an extrinsic error with a time offset.

## Preliminary calibration result (20261030_065908)

- IMU static: gyro bias ~1e-4 rad/s, `|acc| = 9.806 m/s2`.
- Wheel speed vs IMU acc_x: `wheel` timestamps are ~65 ms later; use
  `wheel_to_imu_offset_ms = -65`.
- Steering scale: integrated IMU yaw / wheel-model yaw = **0.914**; provisional
  because wheelbase and steering dynamics are coupled.
- Steering feedback is much smoother than wheel speed, so the raw yaw
  cross-correlation peak is broad (460–710 ms); it is not a reliable pure
  timestamp offset.
- LiDAR-IMU time offset: C16 scan-to-scan azimuth-profile matching (2290 pairs,
  median confidence 0.966) gives raw -40 ms, detrended/derivative -50 ms.
  Use **-50 ms** as the initial value; verify the sign in the chosen LIO.
- Translation/rotation are the legacy initial values plus the 180 deg IMU roll.
  They are not yet optimized by a full LiDAR-IMU extrinsic solver.

### LiDAR-IMU offset without SLAM

`tools/estimate_lidar_imu_offset.py` builds a 360-degree azimuth occupancy
profile per C16 scan, finds the circular shift between consecutive scans, and
correlates the resulting yaw rate with gyro Z:

```bash
python3 /ws/tools/estimate_lidar_imu_offset.py /bags/<bag> --max-lag-ms 2000
```

The sign convention is: positive = add this many ms to LiDAR timestamps to
align with the IMU. The output includes the scan-match confidence and the top
correlation peaks, so a broad/ambiguous result can be rejected. This is a
time-offset tool only; it does not refine extrinsic translation/rotation.


## 1. Reduce FTDI batching

The VX1000 uses an FTDI FT232R. Linux defaults `latency_timer` to 16 ms, which
batches 100 Hz IMU frames. Set it to 1 ms on the vehicle host:

```bash
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
```

Make it persistent with the rule in
[`deploy/udev/99-smartcar-ftdi.rules`](../deploy/udev/99-smartcar-ftdi.rules):

```bash
sudo cp deploy/udev/99-smartcar-ftdi.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=usb-serial
```

Re-plug the VX1000 and verify the value is `1`.

## 2. Record on the vehicle

Start the sensor and chassis stacks first:

```bash
docker compose -f deploy/compose.vehicle.yaml up -d sensors chassis
```

Then record with the `record` profile:

```bash
docker compose -f deploy/compose.vehicle.yaml --profile record run --rm record
```

The recorder writes MCAP + zstd to `SMARTCAR_BAGS_HOST_DIR`
(default `/home/yunle/bags`), splits at `MAX_BAG_SIZE_GB`, refuses to start
below `MIN_FREE_GB`, and stops cleanly on Ctrl-C so the MCAP metadata is
finalized.

Default topics:

```text
/lslidar_point_cloud
/imu/data
/VX1000_imu
/vehicle_status
/chassis/odom
/tf
/tf_static
```

A time-offset monitor is started automatically and writes
`<bag>_sync/sync_<timestamp>.csv` with arrival time, header stamp, latency and
frame id for each message. It does **not** rewrite timestamps.

Override the topic list with `RECORD_TOPICS`, or disable the monitor with
`START_SYNC_MONITOR=0`.

## 3. Calibration bag

A static bag cannot estimate time offsets. Record 2–3 minutes with:

1. 10 s stationary;
2. gentle acceleration and braking;
3. 2–3 left and right turns at low/medium speed;
4. 10 s stationary at the end.

Keep the LiDAR and IMU topics in the same bag. Avoid bumps and hard stops.

## 4. Estimate wheel-odometry to IMU offset

Copy the bag to the dev machine. In the dev container:

```bash
python3 /ws/tools/estimate_offsets.py /bags/<bag> --wheel-base 1.2
```

For a smoother steering bag the raw yaw peak is broad; `analyze_calibration.py`
also checks the sharper acceleration events (speed derivative vs IMU acc_x)
and fits the steer scale:

```bash
python3 /ws/tools/analyze_calibration.py /bags/<bag> --wheel-base 1.2
```

The simpler estimator reconstructs chassis yaw rate from speed + steering and
finds the lag that best correlates it with gyro Z. It reports:

```text
best wheel-IMU lag: +X ms (correlation 0.xx)
convention: positive = add this many ms to wheel timestamps to align with IMU
```

If the correlation is low or the best lag is at the search boundary, the bag
did not contain enough motion; re-record.

Write the result to `config/calibration/time_offsets.yaml`:

```yaml
wheel_to_imu_offset_ms: X
```

## 5. Estimate LiDAR to IMU offset

This offset cannot be recovered from the wheel and IMU topics alone. Use the
LIO front-end with the same calibration bag:

1. Sweep `time_offset_lidar_to_imu` (or the equivalent parameter) over a small
   range, e.g. `-100 ms … +100 ms` in 10 ms steps.
2. Run offline LIO on the bag for each value.
3. Pick the value with the least distorted map / smoothest trajectory /
   lowest point-to-map residual.
4. Store it in `config/calibration/time_offsets.yaml` and the LIO parameter
   file.

The sign convention depends on the LIO implementation; verify with a straight
line and a rotation, not only a static scene.

## 6. Applying the offsets

- Keep the raw topics unchanged; do not republish time-shifted sensor data.
- LIO consumes `lidar_to_imu_offset_ms` through its own parameter.
- Wheel fusion / odometry consumers consume `wheel_to_imu_offset_ms`.
- Record the bag name, date, LIO version and offset values in the release
  manifest.

## Limitations

- No PPS/PTP: offsets are constant estimates plus residual jitter.
- FTDI scheduling and Wi-Fi/campus traffic do not affect the vehicle-local bag,
  but CPU load can increase timestamp jitter.
- The host clock must not step during a run (use chrony slew, not step).
- For long baselines or multiple machines, hardware time sync is still the
  proper solution.
