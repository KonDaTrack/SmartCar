#!/usr/bin/env python3
"""Deeper calibration analysis for the chassis/IMU calibration bag.

Outputs:
  - IMU static bias and gravity direction
  - wheel-IMU lag from raw, detrended and derivative signals
  - steer scale correction via grid search

This is a diagnostic companion to estimate_offsets.py. It does not estimate
LiDAR-IMU offset; that needs a LiDAR-inertial calibration/front-end.

Usage:
  python3 tools/analyze_calibration.py /bags/<bag> --wheel-base 1.2
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
            return reader
        except RuntimeError as exc:
            last_error = exc
    raise RuntimeError(f'cannot open bag {bag_path}: {last_error}')


def stamp_seconds(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def read_bag(path):
    reader = open_reader(path)
    rows = {
        'wt': [], 'speed': [], 'steer': [], 'gear': [],
        'it': [], 'gx': [], 'gy': [], 'gz': [],
        'ax': [], 'ay': [], 'az': [],
    }
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == '/vehicle_status':
            msg = deserialize_message(data, VehicleStatus)
            rows['wt'].append(stamp_seconds(msg))
            rows['speed'].append(float(msg.speed))
            rows['steer'].append(float(msg.steer))
            rows['gear'].append(int(msg.gear))
        elif topic == '/imu/data':
            msg = deserialize_message(data, Imu)
            rows['it'].append(stamp_seconds(msg))
            rows['gx'].append(float(msg.angular_velocity.x))
            rows['gy'].append(float(msg.angular_velocity.y))
            rows['gz'].append(float(msg.angular_velocity.z))
            rows['ax'].append(float(msg.linear_acceleration.x))
            rows['ay'].append(float(msg.linear_acceleration.y))
            rows['az'].append(float(msg.linear_acceleration.z))
    if len(rows['wt']) < 100 or len(rows['it']) < 100:
        raise RuntimeError('not enough wheel/IMU messages')
    for key in rows:
        rows[key] = np.asarray(rows[key])
    return rows


def moving_average(x, window):
    if window <= 1:
        return x.copy()
    kernel = np.ones(window) / window
    return np.convolve(x, kernel, mode='same')


def resample(rows, rate, wheel_base):
    wt, it = rows['wt'], rows['it']
    order = np.argsort(wt)
    wt = wt[order]
    speed = rows['speed'][order]
    steer = rows['steer'][order]
    gear = rows['gear'][order]
    order = np.argsort(it)
    it = it[order]
    gz = rows['gz'][order]
    ax = rows['ax'][order]
    ay = rows['ay'][order]
    az = rows['az'][order]

    t0, t1 = max(wt[0], it[0]), min(wt[-1], it[-1])
    grid = np.arange(t0, t1, 1.0 / rate)
    v = np.interp(grid, wt, speed)
    st = np.interp(grid, wt, steer)
    gr = np.interp(grid, wt, gear)
    imu = np.interp(grid, it, gz)
    acc = np.array([
        np.interp(grid, it, ax),
        np.interp(grid, it, ay),
        np.interp(grid, it, az),
    ])

    v = np.where(gr == 3, -v, np.where(gr == 1, v, 0.0))
    wheel_yaw = v * np.tan(np.radians(st)) / wheel_base
    return grid, v, st, wheel_yaw, imu, acc


def pearson_curve(a, b, max_lag):
    n = len(a)
    lags = np.arange(-max_lag, max_lag + 1)
    values = np.zeros(len(lags))
    for idx, lag in enumerate(lags):
        if lag >= 0:
            x = a[:n - lag] if lag else a
            y = b[lag:]
        else:
            x = a[-lag:]
            y = b[:n + lag]
        if len(x) < 100:
            continue
        x = x - x.mean()
        y = y - y.mean()
        denom = np.linalg.norm(x) * np.linalg.norm(y)
        if denom > 1e-9:
            values[idx] = float(np.dot(x, y) / denom)
    return lags, values


def top_peaks(lags, values, rate, count=5, min_sep_ms=100):
    order = np.argsort(-np.abs(values))
    peaks = []
    for idx in order:
        lag_ms = lags[idx] / rate * 1000.0
        if any(abs(lag_ms - p[0]) < min_sep_ms for p in peaks):
            continue
        peaks.append((lag_ms, float(values[idx])))
        if len(peaks) >= count:
            break
    return peaks


def fit_steer_scale(v, st, imu, rate, best_lag_ms, wheel_base):
    best = None
    for scale in np.arange(-2.0, 2.0001, 0.005):
        angle = np.radians(st * scale)
        angle = np.clip(angle, -1.4, 1.4)
        model = v * np.tan(angle) / wheel_base
        model = model - moving_average(model, int(rate * 2.0))
        target = imu - moving_average(imu, int(rate * 2.0))
        lag = int(round(best_lag_ms / 1000.0 * rate))
        n = len(model)
        if lag >= 0:
            a = model[:n - lag] if lag else model
            b = target[lag:]
        else:
            a = model[-lag:]
            b = target[:n + lag]
        if len(a) < 100:
            continue
        a = a - a.mean()
        b = b - b.mean()
        denom = np.linalg.norm(a) * np.linalg.norm(b)
        if denom < 1e-9:
            continue
        corr = float(np.dot(a, b) / denom)
        if best is None or abs(corr) > abs(best[1]):
            best = (float(scale), corr)
    return best


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag')
    parser.add_argument('--wheel-base', type=float, default=1.2)
    parser.add_argument('--rate', type=float, default=200.0)
    parser.add_argument('--max-lag-ms', type=float, default=2000.0)
    args = parser.parse_args()

    rows = read_bag(args.bag)
    rate = args.rate
    max_lag = int(args.max_lag_ms / 1000.0 * rate)
    grid, v, st, wheel_yaw, imu, acc = resample(rows, rate, args.wheel_base)
    print(f'duration={grid[-1] - grid[0]:.1f}s samples={len(grid)}')
    print(f'wheel yaw std={np.std(wheel_yaw):.3f} rad/s '
          f'imu gz std={np.std(imu):.3f} rad/s')

    wheel_dt = wheel_yaw - moving_average(wheel_yaw, int(rate * 2.0))
    imu_dt = imu - moving_average(imu, int(rate * 2.0))
    wheel_d = np.gradient(wheel_dt)
    imu_d = np.gradient(imu_dt)

    for name, a, b in (
        ('raw', wheel_yaw, imu),
        ('detrended', wheel_dt, imu_dt),
        ('derivative', wheel_d, imu_d),
    ):
        lags, values = pearson_curve(a, b, max_lag)
        best = int(np.argmax(np.abs(values)))
        print(f'\n{name}: best lag={lags[best] / rate * 1000:+.1f} ms '
              f'corr={values[best]:+.3f}')
        for lag_ms, corr in top_peaks(lags, values, rate):
            print(f'  peak {lag_ms:+8.1f} ms  corr {corr:+.3f}')

    # Use the detrended best lag for the scale fit.
    lags, values = pearson_curve(wheel_dt, imu_dt, max_lag)
    lag_ms = lags[int(np.argmax(np.abs(values)))] / rate * 1000.0
    scale = fit_steer_scale(v, st, imu, rate, lag_ms, args.wheel_base)
    if scale:
        print(f'\nsteer scale at lag {lag_ms:+.1f} ms: {scale[0]:+.3f} '
              f'(corr {scale[1]:+.3f})')
        print('  apply as steer_scale (chassis) / steer_angle_scale (odometry)')

    # Least-squares scale on the raw signals (less sensitive to lag).
    lag = int(round(lag_ms / 1000.0 * rate))
    n = len(wheel_yaw)
    if lag >= 0:
        wy = wheel_yaw[:n - lag] if lag else wheel_yaw
        iz = imu[lag:]
    else:
        wy = wheel_yaw[-lag:]
        iz = imu[:n + lag]
    wy = wy - wy.mean()
    iz = iz - iz.mean()
    denom = float(np.dot(wy, wy))
    if denom > 1e-9:
        k = float(np.dot(wy, iz) / denom)
        print(f'raw least-squares wheel scale: {k:+.3f} '
              f'(1.0 = current steer calibration)')
        print(f'raw correlation at zero lag: '
              f'{np.corrcoef(wheel_yaw, imu)[0, 1]:+.3f}')

    # Acceleration events are sharper than smooth steering; use them to
    # identify the forward IMU axis and a less ambiguous time lag.
    speed_smooth = moving_average(v, max(1, int(rate * 0.2)))
    speed_dot = np.gradient(speed_smooth, 1.0 / rate)
    print('\nlongitudinal acceleration vs IMU axes:')
    acc_lag_ms = None
    acc_best_corr = 0.0
    for name, axis in zip(('x', 'y', 'z'), acc):
        lags, values = pearson_curve(speed_dot, axis, max_lag)
        best = int(np.argmax(np.abs(values)))
        lag_value = lags[best] / rate * 1000.0
        corr_value = float(values[best])
        print(f'  acc_{name}: best lag={lag_value:+.1f} ms '
              f'corr={corr_value:+.3f}')
        if abs(corr_value) > abs(acc_best_corr):
            acc_best_corr = corr_value
            acc_lag_ms = lag_value

    if acc_lag_ms is not None:
        print(f'\nprimary wheel-IMU lag from acceleration: {acc_lag_ms:+.1f} ms '
              f'(corr {acc_best_corr:+.3f})')
        scale2 = fit_steer_scale(
            v, st, imu, rate, acc_lag_ms, args.wheel_base)
        if scale2:
            print(f'steer scale at acceleration lag: {scale2[0]:+.3f} '
                  f'(corr {scale2[1]:+.3f})')

        lag = int(round(acc_lag_ms / 1000.0 * rate))
        n = len(wheel_yaw)
        if lag >= 0:
            wy = wheel_yaw[:n - lag] if lag else wheel_yaw
            iz = imu[lag:]
        else:
            wy = wheel_yaw[-lag:]
            iz = imu[:n + lag]
        wy = wy - wy.mean()
        iz = iz - iz.mean()
        denom = float(np.dot(wy, wy))
        if denom > 1e-9:
            print(f'raw wheel scale at acceleration lag: '
                  f'{float(np.dot(wy, iz) / denom):+.3f}')

    int_wheel = float(np.trapz(wheel_yaw, grid))
    int_imu = float(np.trapz(imu, grid))
    if abs(int_wheel) > 1e-3:
        print(f'integral yaw ratio imu/wheel: {int_imu / int_wheel:+.3f} '
              f'(wheel={int_wheel:+.3f} rad, imu={int_imu:+.3f} rad)')

    it = rows['it']
    for label, mask in (
        ('start', it < it[0] + 10.0),
        ('end', it > it[-1] - 10.0),
    ):
        gx = rows['gx'][mask]
        gy = rows['gy'][mask]
        gz = rows['gz'][mask]
        ax = rows['ax'][mask]
        ay = rows['ay'][mask]
        az = rows['az'][mask]
        if len(gx) == 0:
            continue
        gyro = np.array([gx.mean(), gy.mean(), gz.mean()])
        acc = np.array([ax.mean(), ay.mean(), az.mean()])
        print(f'\nIMU {label} 10s: gyro mean={gyro} rad/s '
              f'std={np.std(gz):.3f}')
        print(f'  acc mean={acc} m/s2 |acc|={np.linalg.norm(acc):.3f}')


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:  # noqa: BLE001
        print(f'ERROR: {exc}', file=sys.stderr)
        sys.exit(1)
