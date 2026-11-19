#!/usr/bin/env python3
"""Compare a TUM trajectory against the bag's chassis odometry over time."""
import sys
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader


def main():
    bag, tum = sys.argv[1], sys.argv[2]
    odom = []
    with AnyReader([Path(bag)]) as reader:
        for conn, _t, raw in reader.messages():
            if conn.topic != '/chassis/odom':
                continue
            msg = reader.deserialize(raw, conn.msgtype)
            s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            p = msg.pose.pose.position
            odom.append((s, p.x, p.y, p.z))
    odom = np.asarray(odom)
    traj = np.loadtxt(tum, comments='#')
    t = traj[:, 0]
    p = traj[:, 1:4]
    # Normalize both to their first sample.
    odom_p = odom[:, 1:4] - odom[0, 1:4]
    traj_p = p - p[0]
    print(f'odom: {len(odom)} samples, path {np.linalg.norm(np.diff(odom_p,axis=0),axis=1).sum():.2f} m')
    print(f'traj: {len(traj)} samples, path {np.linalg.norm(np.diff(traj_p,axis=0),axis=1).sum():.2f} m')
    print('  t_rel   odom_xy           traj_xy           ratio')
    for dt in [0, 5, 10, 15, 20, 25, 30, 35, 40]:
        tt = t[0] + dt
        if tt > t[-1]:
            break
        oi = np.argmin(np.abs(odom[:, 0] - tt))
        ti = np.argmin(np.abs(t - tt))
        o = odom_p[oi]
        q = traj_p[ti]
        ratio = np.linalg.norm(q) / max(np.linalg.norm(o), 1e-6)
        print(f'  {dt:5.0f}   {o[0]:+7.2f},{o[1]:+7.2f}   '
              f'{q[0]:+7.2f},{q[1]:+7.2f}   {ratio:5.2f}')


if __name__ == '__main__':
    main()
