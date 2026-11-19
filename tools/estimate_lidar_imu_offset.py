#!/usr/bin/env python3
"""Estimate LiDAR-IMU time offset from a C16 + VX1000 bag.

Method: build a 360-degree azimuth occupancy profile for each C16 scan, find
the circular shift between consecutive scans by FFT cross-correlation, convert
the shift to a yaw rate, then correlate that with gyro Z. No SLAM front-end is
required.

Limitations:
  - 10 Hz LiDAR gives a coarse yaw-rate signal; use a bag with clear turns.
  - Translation and changing scene content add noise; confidence is printed.
  - This estimates time offset only. Use the static TF / LiDAR-IMU extrinsics
    from calibration for the transform.

Usage:
  python3 tools/estimate_lidar_imu_offset.py /bags/<bag> --max-lag-ms 2000
"""

import argparse
import math
import sys

import numpy as np
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from sensor_msgs.msg import Imu, PointCloud2

# lslidar_driver XYZIRT cloud: point_step=32, offsets as observed.
CLOUD_DTYPE = np.dtype({
    'names': ['x', 'y', 'z', 'intensity', 'ring', 'time'],
    'formats': ['<f4', '<f4', '<f4', '<f4', '<u2', '<f4'],
    'offsets': [0, 4, 8, 16, 20, 24],
    'itemsize': 32,
})
AZ_BINS = 3600  # 0.1 deg


def open_reader(path):
    last_error = None
    for storage_id in ('mcap', 'sqlite3'):
        reader = SequentialReader()
        try:
            reader.open(
                StorageOptions(uri=path, storage_id=storage_id),
                ConverterOptions('', ''))
            return reader
        except RuntimeError as exc:
            last_error = exc
    raise RuntimeError(f'cannot open bag {path}: {last_error}')


def stamp_seconds(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def profile_from_cloud(msg):
    points = np.frombuffer(msg.data, dtype=CLOUD_DTYPE)
    x = points['x'].astype(np.float64)
    y = points['y'].astype(np.float64)
    z = points['z'].astype(np.float64)
    r = np.hypot(x, y)
    mask = (r > 2.0) & (r < 60.0) & np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
    x, y, r = x[mask], y[mask], r[mask]
    if len(x) < 200:
        return None
    az = np.arctan2(y, x)
    bins = np.floor((az + math.pi) / (2.0 * math.pi) * AZ_BINS).astype(np.int64)
    bins %= AZ_BINS
    # Far points rotate more coherently with yaw than near points.
    weights = np.clip(r / 30.0, 0.2, 1.0)
    profile = np.bincount(bins, weights=weights, minlength=AZ_BINS)
    total = profile.sum()
    if total <= 0:
        return None
    return (profile / total).astype(np.float32)


def circular_shift(a, b):
    """Return shift (bins) that best aligns a to b, and normalized score."""
    a = a - a.mean()
    b = b - b.mean()
    denom = float(np.linalg.norm(a) * np.linalg.norm(b))
    if denom < 1e-12:
        return 0, 0.0
    corr = np.fft.ifft(np.fft.fft(a) * np.conj(np.fft.fft(b))).real
    idx = int(np.argmax(corr))
    shift = idx if idx <= AZ_BINS // 2 else idx - AZ_BINS
    return shift, float(corr[idx] / denom)


def read_bag(path):
    reader = open_reader(path)
    profiles = []
    lidar_t = []
    imu_t = []
    imu_gz = []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == '/lslidar_point_cloud':
            msg = deserialize_message(data, PointCloud2)
            profile = profile_from_cloud(msg)
            if profile is not None:
                profiles.append(profile)
                lidar_t.append(stamp_seconds(msg))
        elif topic == '/imu/data':
            msg = deserialize_message(data, Imu)
            imu_t.append(stamp_seconds(msg))
            imu_gz.append(float(msg.angular_velocity.z))
    if len(profiles) < 20 or len(imu_t) < 100:
        raise RuntimeError('not enough LiDAR/IMU data')
    return (np.asarray(lidar_t), np.asarray(profiles),
            np.asarray(imu_t), np.asarray(imu_gz))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag')
    parser.add_argument('--max-lag-ms', type=float, default=2000.0)
    parser.add_argument('--min-conf', type=float, default=0.05)
    args = parser.parse_args()

    lidar_t, profiles, imu_t, imu_gz = read_bag(args.bag)
    order = np.argsort(lidar_t)
    lidar_t, profiles = lidar_t[order], profiles[order]
    order = np.argsort(imu_t)
    imu_t, imu_gz = imu_t[order], imu_gz[order]

    lidar_rate = []
    lidar_mid = []
    confs = []
    for i in range(1, len(lidar_t)):
        dt = lidar_t[i] - lidar_t[i - 1]
        if not (0.05 < dt < 0.2):
            continue
        shift, conf = circular_shift(profiles[i - 1], profiles[i])
        # The shift direction is arbitrary until correlated with the IMU.
        # Area per bin is 2*pi/AZ_BINS.
        yaw = shift * (2.0 * math.pi / AZ_BINS) / dt
        lidar_rate.append(yaw)
        lidar_mid.append(0.5 * (lidar_t[i] + lidar_t[i - 1]))
        confs.append(conf)
    lidar_rate = np.asarray(lidar_rate)
    lidar_mid = np.asarray(lidar_mid)
    confs = np.asarray(confs)
    good = confs >= args.min_conf
    print(f'LiDAR scans={len(lidar_t)} pairs={len(lidar_rate)} '
          f'good_pairs={int(good.sum())} '
          f'conf median={np.median(confs):.3f} max={confs.max():.3f}')
    if good.sum() < 20:
        print('WARNING: too few confident scan matches; result is unreliable')
    lidar_rate = lidar_rate[good]
    lidar_mid = lidar_mid[good]

    # Interpolate IMU gyro Z at the LiDAR pair midpoints.
    imu_at_lidar = np.interp(lidar_mid, imu_t, imu_gz)
    print(f'lidar yaw-rate std={np.std(lidar_rate):.3f} rad/s '
          f'imu gz std={np.std(imu_at_lidar):.3f} rad/s')

    # Put both on a regular grid so lag resolution is not limited by 10 Hz.
    rate = 100.0
    t0, t1 = max(lidar_mid[0], imu_t[0]), min(lidar_mid[-1], imu_t[-1])
    grid = np.arange(t0, t1, 1.0 / rate)
    lr = np.interp(grid, lidar_mid, lidar_rate)
    iz = np.interp(grid, imu_t, imu_gz)
    lr = lr - lr.mean()
    iz = iz - iz.mean()

    # The circular-shift sign is a convention; align it to the IMU. This does
    # not change the time lag, only the sign of the yaw-rate signal.
    if np.dot(lr, iz) < 0.0:
        lr = -lr
        print('flipped LiDAR yaw-rate sign to match IMU')

    max_lag = int(args.max_lag_ms / 1000.0 * rate)

    def correlate(a, b, name):
        n = len(a)
        best_lag, best_corr = 0, 0.0
        peaks = []
        for lag in range(-max_lag, max_lag + 1):
            if lag >= 0:
                aa = a[:n - lag] if lag else a
                bb = b[lag:]
            else:
                aa = a[-lag:]
                bb = b[:n + lag]
            if len(aa) < rate:
                continue
            aa = aa - aa.mean()
            bb = bb - bb.mean()
            denom = float(np.linalg.norm(aa) * np.linalg.norm(bb))
            if denom < 1e-9:
                continue
            corr = float(np.dot(aa, bb) / denom)
            peaks.append((lag / rate * 1000.0, corr))
            if abs(corr) > abs(best_corr):
                best_corr, best_lag = corr, lag
        peaks.sort(key=lambda x: -abs(x[1]))
        print(f'\n{name}: best lag={best_lag / rate * 1000:+.1f} ms '
              f'corr={best_corr:+.3f}')
        shown = []
        for lag_ms, corr in peaks:
            if any(abs(lag_ms - p) < 50.0 for p in shown):
                continue
            shown.append(lag_ms)
            print(f'  {lag_ms:+8.1f} ms  corr {corr:+.3f}')
            if len(shown) >= 5:
                break
        return best_lag / rate * 1000.0

    raw_lag = correlate(lr, iz, 'raw')

    def moving_average(x, window):
        kernel = np.ones(window) / window
        return np.convolve(x, kernel, mode='same')

    lr_hp = lr - moving_average(lr, int(rate * 1.0))
    iz_hp = iz - moving_average(iz, int(rate * 1.0))
    hp_lag = correlate(lr_hp, iz_hp, 'detrended (1 s)')

    lr_d = np.gradient(lr_hp)
    iz_d = np.gradient(iz_hp)
    correlate(lr_d, iz_d, 'derivative')

    print('\nconvention: positive = add this many ms to LiDAR timestamps to align with IMU')
    print(f'raw={raw_lag:+.1f} ms, detrended={hp_lag:+.1f} ms')


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:  # noqa: BLE001
        print(f'ERROR: {exc}', file=sys.stderr)
        sys.exit(1)
