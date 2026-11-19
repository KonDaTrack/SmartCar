#!/usr/bin/env python3
"""Prepare a SLAM bench bag from the recorded SmartCar bag.

The C16 driver publishes:
  header.stamp = sweep end
  point.time   = 0..scan_duration relative to the sweep start

Algorithms disagree about that convention, and the recorded bag has no /tf
(chassis odometry TF was disabled). This script writes a new uncompressed MCAP
with:

  * /lslidar_point_cloud header moved to the sweep start (point.time unchanged)
  * /chassis/odom and a synthesized /tf odom->base_link
  * /imu/data and /tf_static copied as-is

Usage:
  prepare_bag.py <src_bag> <out_bag> [--max-seconds N]
"""

import argparse
import shutil
import sys
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.rosbag2 import StoragePlugin, Writer
from rosbags.typesys import Stores, get_typestore

FMT = {1: 'i1', 2: 'u1', 3: 'i2', 4: 'u2', 5: 'i4', 6: 'u4', 7: 'f4', 8: 'f8'}
KEEP = {'/lslidar_point_cloud', '/imu/data', '/chassis/odom', '/tf_static'}


def stamp_ns(msg):
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec


def set_stamp(msg, ns):
    msg.header.stamp.sec = int(ns // 1_000_000_000)
    msg.header.stamp.nanosec = int(ns % 1_000_000_000)


def cloud_time_range(msg):
    names, formats, offsets = [], [], []
    for f in msg.fields:
        if f.count != 1:
            continue
        names.append(f.name)
        formats.append(FMT[f.datatype])
        offsets.append(f.offset)
    dt = np.dtype({'names': names, 'formats': formats,
                   'offsets': offsets, 'itemsize': msg.point_step})
    pts = np.frombuffer(msg.data, dtype=dt)
    if 'time' not in pts.dtype.names or len(pts) == 0:
        return None
    t = pts['time'].astype(np.float64)
    return float(t.min()), float(t.max())


def make_tf(typestore, odom):
    tf = typestore.types['tf2_msgs/msg/TFMessage'](
        transforms=[
            typestore.types['geometry_msgs/msg/TransformStamped'](
                header=typestore.types['std_msgs/msg/Header'](
                    stamp=odom.header.stamp,
                    frame_id=odom.header.frame_id,
                ),
                child_frame_id=odom.child_frame_id,
                transform=typestore.types['geometry_msgs/msg/Transform'](
                    translation=typestore.types['geometry_msgs/msg/Vector3'](
                        x=odom.pose.pose.position.x,
                        y=odom.pose.pose.position.y,
                        z=odom.pose.pose.position.z,
                    ),
                    rotation=typestore.types['geometry_msgs/msg/Quaternion'](
                        x=odom.pose.pose.orientation.x,
                        y=odom.pose.pose.orientation.y,
                        z=odom.pose.pose.orientation.z,
                        w=odom.pose.pose.orientation.w,
                    ),
                ),
            )
        ]
    )
    return tf


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('src')
    parser.add_argument('out')
    parser.add_argument('--max-seconds', type=float, default=0.0)
    args = parser.parse_args()

    src = Path(args.src)
    out = Path(args.out)
    if out.exists():
        shutil.rmtree(out)

    typestore = get_typestore(Stores.ROS2_HUMBLE)
    counts = {}
    shifted = []
    t0 = None
    static_pending = []

    with AnyReader([src]) as reader, Writer(
            out, version=9, storage_plugin=StoragePlugin.MCAP) as writer:
        conns = {}
        for topic, msgtype in (
            ('/lslidar_point_cloud', 'sensor_msgs/msg/PointCloud2'),
            ('/imu/data', 'sensor_msgs/msg/Imu'),
            ('/chassis/odom', 'nav_msgs/msg/Odometry'),
            ('/tf', 'tf2_msgs/msg/TFMessage'),
            ('/tf_static', 'tf2_msgs/msg/TFMessage'),
        ):
            conns[topic] = writer.add_connection(topic, msgtype, typestore=typestore)

        for i, (conn, timestamp, raw) in enumerate(reader.messages()):
            if conn.topic not in KEEP:
                continue
            msg = reader.deserialize(raw, conn.msgtype)
            if conn.topic == '/tf_static':
                # Static transforms are timeless; write them at the first
                # non-static header stamp so the bag does not span the gap
                # between replay time and recorded header time.
                if t0 is None:
                    static_pending.append((conn.topic, msg))
                else:
                    writer.write(conns[conn.topic], t0,
                                 typestore.serialize_cdr(msg, 'tf2_msgs/msg/TFMessage'))
                    counts[conn.topic] = counts.get(conn.topic, 0) + 1
                continue
            else:
                ts = stamp_ns(msg)
            if t0 is None:
                t0 = ts
                for topic, static_msg in static_pending:
                    writer.write(conns[topic], t0,
                                 typestore.serialize_cdr(static_msg, 'tf2_msgs/msg/TFMessage'))
                    counts[topic] = counts.get(topic, 0) + 1
                static_pending.clear()

            if conn.topic == '/lslidar_point_cloud':
                tr = cloud_time_range(msg)
                if tr is not None:
                    tmin, tmax = tr
                    duration_ns = int(round((tmax - tmin) * 1e9))
                    set_stamp(msg, stamp_ns(msg) - duration_ns)
                    shifted.append(duration_ns * 1e-9)
                ts = stamp_ns(msg)
                writer.write(conns[conn.topic], ts,
                             typestore.serialize_cdr(msg, conn.msgtype))
                counts[conn.topic] = counts.get(conn.topic, 0) + 1

            elif conn.topic == '/chassis/odom':
                ts = stamp_ns(msg)
                writer.write(conns[conn.topic], ts,
                             typestore.serialize_cdr(msg, conn.msgtype))
                tf = make_tf(typestore, msg)
                writer.write(conns['/tf'], ts,
                             typestore.serialize_cdr(tf, 'tf2_msgs/msg/TFMessage'))
                counts[conn.topic] = counts.get(conn.topic, 0) + 1
                counts['/tf'] = counts.get('/tf', 0) + 1

            else:
                writer.write(conns[conn.topic], ts,
                             typestore.serialize_cdr(msg, conn.msgtype))
                counts[conn.topic] = counts.get(conn.topic, 0) + 1

            if args.max_seconds > 0 and conn.topic != '/tf_static' \
                    and (ts - t0) > args.max_seconds * 1e9:
                break
            if i % 20000 == 0:
                print(f'  {i} messages...', flush=True)

    # rosbags writes an empty list for offered_qos_profiles, but rosbag2
    # Humble expects the YAML-decoded string field. Normalize the metadata.
    meta = out / 'metadata.yaml'
    if meta.exists():
        text = meta.read_text()
        text = text.replace('offered_qos_profiles: []', 'offered_qos_profiles: ""')
        text = text.replace('custom_data: null', 'custom_data: {}')
        meta.write_text(text)

    print('wrote', out)
    print('messages:', counts)
    if shifted:
        print(f'C16 header shift: median {np.median(shifted)*1e3:.1f} ms, '
              f'min {min(shifted)*1e3:.1f}, max {max(shifted)*1e3:.1f}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
