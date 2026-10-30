# Time sync and vehicle recording

Scope: LIO sensors only — **LSLIDAR C16, VX1000 IMU and the JD01 chassis
odometry**. The 2D LiDARs and the RealSense camera are not recorded.

## Current time bases

| Topic | Stamp source | Notes |
|---|---|---|
| `/lslidar_point_cloud` | host `now()` at packet receive; cloud header = sweep end | `use_time_service: false`; per-point `time` field is present (`pcl_type: false`) |
| `/imu/data` | host `now()` after serial parse | VX1000 over FTDI at 460800 |
| `/vehicle_status` | host `now()` on UDP receive | `.110` gateway, 100 Hz |
| `/odom` | copies `/vehicle_status` stamp | Ackermann integration |

All three share the **host clock**, so there is no relative clock drift between
them. What remains is transport/processing latency and jitter. This is what the
soft-sync tools measure; it is not a replacement for PPS/PTP.

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
/odom
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

The tool reconstructs chassis yaw rate from speed + steering and finds the lag
that best correlates it with gyro Z. It reports:

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
