#!/usr/bin/env python3
"""Estimate the wheel-odometry to IMU time offset from a calibration bag.

The bag must contain:
  /vehicle_status  smartcar_msgs/VehicleStatus
  /imu/data        sensor_msgs/Imu

Method: rebuild the chassis yaw rate from speed + steering with the Ackermann
model, then find the lag that best correlates it with gyro Z. The vehicle must
actually accelerate/decelerate and turn; a static bag cannot calibrate time.

Positive result means: add that many milliseconds to the wheel timestamps to
align them with the IMU. Write the value to config/calibration/time_sync.yaml.

The lidar-to-IMU offset is not derivable from these two topics; tune the LIO
front-end's time_offset_lidar_to_imu parameter with this bag instead.

Usage:
  python3 tools/estimate_offsets.py /bags/20261030_120000 --wheel-base 1.2
"""

import argparse
import math
import sys

import numpy as np
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from sensor_msgs.msg import Imu
from smartcar_msgs.msg import VehicleStatus


def open_reader(bag_path):
    last_error = None
    for storage_id in ('mcap', 'sqlite3'):
        reader = SequentialReader()
        try:
            reader.open(
                StorageOptions(uri=bag_path, storage_id=storage_id),
                ConverterOptions('', ''))
            return reader, storage_id
        except RuntimeError as exc:
            last_error = exc
    raise RuntimeError(f'cannot open bag {bag_path}: {last_error}')


def stamp_seconds(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def read_bag(bag_path):
    reader, storage_id = open_reader(bag_path)
    print(f'bag={bag_path} storage={storage_id}')

    wheel_t, wheel_yaw = [], []
    imu_t, imu_yaw_rate = [], []
    wheel_topic = '/vehicle_status'
    imu_topic = '/imu/data'

    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == wheel_topic:
            msg = deserialize_message(data, VehicleStatus)
            v = float(msg.speed)
            if msg.gear == 3:       # R
                v = -v
            elif msg.gear != 1:     # N / unknown
                v = 0.0
            steer = math.radians(float(msg.steer))
            wheel_t.append(stamp_seconds(msg))
            wheel_yaw.append(v * math.tan(steer))
        elif topic == imu_topic:
            msg = deserialize_message(data, Imu)
            imu_t.append(stamp_seconds(msg))
            imu_yaw_rate.append(float(msg.angular_velocity.z))

    if len(wheel_t) < 20 or len(imu_t) < 20:
        raise RuntimeError(
            f'not enough data: wheel={len(wheel_t)} imu={len(imu_t)}; '
            f'expected {wheel_topic} and {imu_topic}')

    wheel_t = np.asarray(wheel_t)
    wheel_yaw = np.asarray(wheel_yaw)
    imu_t = np.asarray(imu_t)
    imu_yaw_rate = np.asarray(imu_yaw_rate)

    order = np.argsort(wheel_t)
    wheel_t, wheel_yaw = wheel_t[order], wheel_yaw[order]
    order = np.argsort(imu_t)
    imu_t, imu_yaw_rate = imu_t[order], imu_yaw_rate[order]

    return wheel_t, wheel_yaw, imu_t, imu_yaw_rate


def estimate(wheel_t, wheel_yaw, imu_t, imu_yaw_rate, wheel_base, rate, max_lag_ms):
    # wheel_yaw currently has v * tan(steer); divide by wheel base.
    wheel_yaw = wheel_yaw / wheel_base

    t0 = max(wheel_t[0], imu_t[0])
    t1 = min(wheel_t[-1], imu_t[-1])
    duration = t1 - t0
    if duration < 2.0:
        raise RuntimeError(f'overlap too short: {duration:.2f} s')

    grid = np.arange(t0, t1, 1.0 / rate)
    wheel = np.interp(grid, wheel_t, wheel_yaw)
    imu = np.interp(grid, imu_t, imu_yaw_rate)

    wheel_std = float(np.std(wheel))
    imu_std = float(np.std(imu))
    print(f'duration={duration:.2f} s samples={len(grid)} '
          f'wheel_yaw_std={wheel_std:.3f} rad/s imu_std={imu_std:.3f} rad/s')
    if wheel_std < 0.01 or imu_std < 0.01:
        print('WARNING: almost no rotation; drive with left/right turns for calibration')

    wheel = wheel - np.mean(wheel)
    imu = imu - np.mean(imu)
    if np.std(wheel) > 1e-9:
        wheel = wheel / np.std(wheel)
    if np.std(imu) > 1e-9:
        imu = imu / np.std(imu)

    max_lag = int(round(max_lag_ms / 1000.0 * rate))
    n = len(grid)
    best_lag = 0
    best_corr = 0.0
    for lag in range(-max_lag, max_lag + 1):
        if lag >= 0:
            a = wheel[:n - lag] if lag else wheel
            b = imu[lag:]
        else:
            a = wheel[-lag:]
            b = imu[:n + lag]
        if len(a) < rate:
            continue
        a = a - np.mean(a)
        b = b - np.mean(b)
        denom = float(np.linalg.norm(a) * np.linalg.norm(b))
        if denom < 1e-9:
            continue
        corr = float(np.dot(a, b) / denom)
        if abs(corr) > abs(best_corr):
            best_corr = corr
            best_lag = lag

    offset_ms = best_lag / rate * 1000.0
    print(f'best wheel-IMU lag: {offset_ms:+.1f} ms '
          f'(correlation {best_corr:+.3f})')
    print('convention: positive = add this many ms to wheel timestamps to align with IMU')
    if abs(best_corr) < 0.3:
        print('WARNING: low correlation; check motion, signs, and topic data')
    if abs(best_lag) >= max_lag:
        print('WARNING: best lag is at the search boundary; increase --max-lag-ms')

    print()
    print('Lidar-IMU offset is a separate calibration:')
    print('  run the offline LIO front-end and sweep time_offset_lidar_to_imu')
    print('  with this bag, then store the best value in config/calibration/time_sync.yaml')
    return offset_ms, best_corr


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', help='rosbag2 directory (mcap or sqlite3)')
    parser.add_argument('--wheel-base', type=float, default=1.2)
    parser.add_argument('--rate', type=float, default=200.0,
                        help='resample rate in Hz')
    parser.add_argument('--max-lag-ms', type=float, default=300.0)
    args = parser.parse_args()

    try:
        data = read_bag(args.bag)
        estimate(*data, args.wheel_base, args.rate, args.max_lag_ms)
    except Exception as exc:  # noqa: BLE001 - command-line tool
        print(f'ERROR: {exc}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
