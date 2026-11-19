#!/usr/bin/env python3
"""Export /chassis/odom from a bag to TUM for comparison."""
import sys
from pathlib import Path

from rosbags.highlevel import AnyReader

bag, out = sys.argv[1], sys.argv[2]
with AnyReader([Path(bag)]) as reader, open(out, 'w') as f:
    for conn, _t, raw in reader.messages():
        if conn.topic != '/chassis/odom':
            continue
        m = reader.deserialize(raw, conn.msgtype)
        s = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        f.write(f'{s:.9f} {p.x:.6f} {p.y:.6f} {p.z:.6f} '
                f'{q.x:.6f} {q.y:.6f} {q.z:.6f} {q.w:.6f}\n')
