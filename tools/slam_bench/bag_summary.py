#!/usr/bin/env python3
"""Summarize the C16/IMU/chassis topics in an MCAP bag.

Reports topic counts, the C16 header-vs-point-time convention, and the
chassis dead-reckoning path so we know whether the recorded run returned
near its start. Run inside the benchmark container:

  python3 /ws/tools/slam_bench/bag_summary.py /bags-bench/20261030_080720
"""

import argparse
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader

FMT = {1: 'i1', 2: 'u1', 3: 'i2', 4: 'u2', 5: 'i4', 6: 'u4', 7: 'f4', 8: 'f8'}


def cloud_dtype(msg):
    names, formats, offsets = [], [], []
    for f in msg.fields:
        if f.count != 1:
            continue
        names.append(f.name)
        formats.append(FMT[f.datatype])
        offsets.append(f.offset)
    return np.dtype({'names': names, 'formats': formats,
                     'offsets': offsets, 'itemsize': msg.point_step})


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('bag')
    parser.add_argument('--clouds', type=int, default=20,
                        help='number of clouds to inspect')
    args = parser.parse_args()

    counts = {}
    cloud_info = []
    odom = []
    imu = []

    with AnyReader([Path(args.bag)]) as reader:
        for conn, _t, raw in reader.messages():
            counts[conn.topic] = counts.get(conn.topic, 0) + 1
            if conn.topic == '/lslidar_point_cloud' and len(cloud_info) < args.clouds:
                msg = reader.deserialize(raw, conn.msgtype)
                dt = cloud_dtype(msg)
                pts = np.frombuffer(msg.data, dtype=dt)
                t = pts['time'].astype(np.float64) if 'time' in pts.dtype.names else None
                cloud_info.append({
                    'stamp': stamp(msg),
                    'n': len(pts),
                    't_min': float(t.min()) if t is not None and len(t) else None,
                    't_max': float(t.max()) if t is not None and len(t) else None,
                    'fields': list(pts.dtype.names),
                })
            elif conn.topic == '/chassis/odom':
                msg = reader.deserialize(raw, conn.msgtype)
                p = msg.pose.pose.position
                odom.append((stamp(msg), p.x, p.y, p.z,
                             msg.twist.twist.linear.x,
                             msg.twist.twist.angular.z))
            elif conn.topic == '/imu/data':
                msg = reader.deserialize(raw, conn.msgtype)
                imu.append((stamp(msg),
                            msg.angular_velocity.x, msg.angular_velocity.y,
                            msg.angular_velocity.z,
                            msg.linear_acceleration.x, msg.linear_acceleration.y,
                            msg.linear_acceleration.z,
                            msg.orientation_covariance[0]))

    print('topics:')
    for topic, n in sorted(counts.items()):
        print(f'  {topic:32s} {n:8d}')

    if cloud_info:
        print('\nC16 clouds (first %d):' % len(cloud_info))
        c = cloud_info[0]
        print(f'  fields: {c["fields"]}')
        print(f'  points/scan: {c["n"]}')
        tmins = [c['t_min'] for c in cloud_info if c['t_min'] is not None]
        tmaxs = [c['t_max'] for c in cloud_info if c['t_max'] is not None]
        if tmins:
            print(f'  point time relative to header: '
                  f'min={np.median(tmins):+.4f} s max={np.median(tmaxs):+.4f} s')
            print(f'  => header is at scan {"END" if np.median(tmaxs) <= 2e-3 else "START"}')

    if odom:
        o = np.asarray(odom)
        xy = o[:, 1:3]
        step = np.linalg.norm(np.diff(xy, axis=0), axis=1)
        print('\nchassis odom:')
        print(f'  messages: {len(o)}  duration: {o[-1,0]-o[0,0]:.1f} s')
        print(f'  path length: {step.sum():.1f} m')
        print(f'  start ({xy[0,0]:+.2f}, {xy[0,1]:+.2f})  '
              f'end ({xy[-1,0]:+.2f}, {xy[-1,1]:+.2f})  '
              f'loop error: {np.linalg.norm(xy[-1]-xy[0]):.1f} m')
        print(f'  max speed: {o[:,4].max():.2f} m/s  '
              f'max |yaw rate|: {np.abs(o[:,5]).max():.3f} rad/s')

    if imu:
        a = np.asarray(imu)
        print('\nIMU:')
        print(f'  messages: {len(a)}  duration: {a[-1,0]-a[0,0]:.1f} s')
        print(f'  orientation unavailable flag (cov[0]): '
              f'{np.median(a[:,7]):.1f}')
        print(f'  median |acc|: {np.linalg.norm(a[:,4:7], axis=1).mean():.3f} m/s2')


if __name__ == '__main__':
    main()
