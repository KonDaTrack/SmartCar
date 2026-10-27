#!/usr/bin/env python3
"""Offline smoke test for smartcar_ins and smartcar_chassis.

Runs both drivers against synthetic data:
  - smartcar_ins: a pseudo-terminal carrying 68-byte VX1000 frames
  - smartcar_chassis: UDP CAN frames to 127.0.0.1:18002

Run inside the dev container:
  python3 /ws/tools/smoke_test.py
"""

import math
import os
import pty
import signal
import socket
import struct
import subprocess
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from smartcar_msgs.msg import (
    Vx1000Cov,
    Vx1000Gnss,
    Vx1000Imu,
    Vx1000Ins,
    Vx1000Odom,
    Vx1000Stat,
    VehicleStatus,
)


class BitWriter:
    def __init__(self, size):
        self.buf = bytearray(size)
        self.pos = 0

    def write(self, value, width):
        value = int(value) & ((1 << width) - 1)
        for i in range(width):
            if (value >> i) & 1:
                self.buf[self.pos >> 3] |= 1 << (self.pos & 7)
            self.pos += 1


def pack(values):
    writer = BitWriter(64)
    for value, width in values:
        writer.write(value, width)
    assert writer.pos == 512, writer.pos
    return bytes(writer.buf)


def vx_frame(kind, payload):
    frame = bytearray([0xAA, 0x55, kind]) + payload
    frame.append(sum(frame) & 0xFF)
    assert len(frame) == 68
    return bytes(frame)


def make_ins():
    return pack([
        (2345, 16), (123456, 32), (0, 16),
        (42, 32), (5, 8), (0, 24),
        (int((39.9 + 90) / 1e-10), 64),
        (int((116.4 + 180) / 1e-10), 64),
        (int((50 + 1000) * 1e3), 24),
        (int((1.5 + 100) * 1e3), 20),
        (int((0.5 + 100) * 1e3), 20),
        (int((0.1 + 100) * 1e3), 20),
        (int((0 + 180) * 1e4), 22),
        (int((0 + 90) * 1e4), 22),
        (int(90 * 1e4), 22), (0, 42),
        (0, 48), (7, 8), (0, 8),
    ])


def make_imu():
    return pack([
        (2345, 16), (123456, 32), (0, 16),
        (int((0 + 500) * 1e5), 28), (int((0 + 500) * 1e5), 28), (0, 8),
        (int((0 + 500) * 1e5), 28), (int((0 + 400) * 1e5), 28), (0, 8),
        (int((0 + 400) * 1e5), 28), (int((9.8 + 400) * 1e5), 28), (0, 8),
        (int((25 + 128) * 100), 16), (int((26 + 128) * 100), 16),
        (1, 2), (1, 1), (1, 1), (0, 1), (0, 1), (0, 1), (0, 1), (0, 2),
        (1, 2), (1, 1), (1, 1), (0, 1), (0, 1), (0, 1), (0, 1), (0, 2),
        (0, 12),
        (0, 64),
        (0, 64),
        (0, 48), (9, 8), (0, 8),
    ])


def make_gnss():
    return pack([
        (2345, 16), (123456, 32), (26, 10), (10, 4), (0, 2),
        (27, 5), (12, 5), (34, 6), (56, 6), (789, 10), (3, 4), (0, 4),
        (1, 8), (18, 8), (15, 8),
        (int((39.9 + 90) / 1e-10), 64),
        (int((116.4 + 180) / 1e-10), 64),
        (int((50 + 1000) * 1e3), 24), (int(90 * 100), 16),
        (int(90 * 100), 16), (0, 8),
        (int((1.5 + 100) * 100), 16), (int((1.5 + 100) * 100), 16),
        (int((0 + 100) * 100), 16), (int((0 + 100) * 100), 16),
        (100, 10), (90, 10), (110, 10),
        (50, 10), (50, 10), (20, 10), (0, 4),
        (0, 2), (1, 1), (1, 1), (1, 1), (1, 1), (1, 1), (1, 1),
        (1, 2), (1, 2), (1, 1), (1, 2), (0, 33), (3, 8), (0, 8),
    ])


def make_odom():
    return pack([
        (2345, 16), (123456, 32), (0, 16),
        (1, 4),
        (int((1.5 + 100) * 1e3), 20),
        (int((1.5 + 100) * 1e3), 20),
        (int((1.5 + 100) * 1e3), 20),
        (int((1.5 + 100) * 1e3), 20),
        (int((1.5 + 100) * 1e3), 20),
        (100, 12), (100, 12),
        (100, 12), (100, 12),
        (int((30 + 720) * 10), 16), (10, 16), (0, 8),
        (int((5 + 5) * 1e5), 20), (int((5 + 5) * 1e5), 20), (0, 24),
        (1, 1), (0, 8), (0, 16), (0, 8), (0, 16), (0, 4), (0, 11),
        (0, 64),
        (0, 48), (11, 8), (0, 8),
    ])


def make_stat():
    return pack([(0, 64), (0, 64), (0, 64), (0, 64),
                 (0, 64), (0, 64), (0, 64), (0, 48), (12, 8), (0, 8)])


def make_cov():
    return pack([(0, 64), (0, 64), (0, 64), (0, 64),
                 (0, 64), (0, 64), (0, 64), (0, 48), (13, 8), (0, 8)])


class Collector(Node):
    def __init__(self):
        super().__init__('smartcar_smoke_test')
        self.ins = None
        self.imu = None
        self.gnss = None
        self.stat = None
        self.cov = None
        self.odom = None
        self.std_imu = None
        self.std_fix = None
        self.status = None
        self.odo = None

        self.create_subscription(Vx1000Ins, '/VX1000_ins', self._set('ins'), 10)
        self.create_subscription(Vx1000Imu, '/VX1000_imu', self._set('imu'), 10)
        self.create_subscription(Vx1000Gnss, '/VX1000_gnss', self._set('gnss'), 10)
        self.create_subscription(Vx1000Stat, '/VX1000_stat', self._set('stat'), 10)
        self.create_subscription(Vx1000Cov, '/VX1000_cov', self._set('cov'), 10)
        self.create_subscription(Vx1000Odom, '/VX1000_odom', self._set('odom'), 10)
        self.create_subscription(Imu, '/imu/data', self._set('std_imu'), 10)
        self.create_subscription(NavSatFix, '/gnss/fix', self._set('std_fix'), 10)
        self.create_subscription(VehicleStatus, '/vehicle_status', self._set('status'), 10)
        self.create_subscription(Odometry, '/odom', self._set('odo'), 10)

    def _set(self, name):
        def cb(msg):
            setattr(self, name, msg)
        return cb


def spin_for(node, seconds, feed=None):
    deadline = time.time() + seconds
    while time.time() < deadline:
        if feed:
            feed()
        rclpy.spin_once(node, timeout_sec=0.02)


def wait_until(node, predicate, seconds, feed=None):
    deadline = time.time() + seconds
    while time.time() < deadline:
        if feed:
            feed()
        rclpy.spin_once(node, timeout_sec=0.02)
        if predicate():
            return True
    return False


def check(condition, text, failures):
    print(('PASS ' if condition else 'FAIL ') + text)
    if not condition:
        failures.append(text)


def close(a, b, tol=1e-3):
    return abs(a - b) <= tol


def stop(proc):
    """Stop a ros2 run process including its child, then return its output."""
    if proc.poll() is None:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except ProcessLookupError:
            pass
    try:
        out, _ = proc.communicate(timeout=5)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass
        out, _ = proc.communicate(timeout=5)
    return out


def test_vx(failures):
    master_fd, slave_fd = pty.openpty()
    port = os.ttyname(slave_fd)
    os.close(slave_fd)

    proc = subprocess.Popen(
        ['ros2', 'run', 'smartcar_ins', 'vx1000_node',
         '--ros-args', '-p', f'port:={port}', '-p', 'baudrate:=460800'],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
        start_new_session=True)

    node = Collector()
    frames = [
        vx_frame(0xA0, make_ins()),
        vx_frame(0xA1, make_imu()),
        vx_frame(0xA2, make_gnss()),
        vx_frame(0xA3, make_stat()),
        vx_frame(0xA4, make_cov()),
        vx_frame(0xA5, make_odom()),
    ]

    def feed():
        for frame in frames:
            os.write(master_fd, frame)
        time.sleep(0.02)

    try:
        wait_until(node, lambda: node.ins is not None, 8.0, feed)
        spin_for(node, 1.0, feed)
    finally:
        node.destroy_node()
        out = stop(proc)

    if node.ins is None:
        print(out[-2000:])
    check(node.ins is not None, 'VX1000 /VX1000_ins received', failures)
    if node.ins:
        check(close(node.ins.ins_lat, 39.9), 'VX1000 latitude 39.9', failures)
        check(close(node.ins.ins_lon, 116.4), 'VX1000 longitude 116.4', failures)
        check(close(node.ins.ins_alt, 50.0), 'VX1000 altitude 50.0', failures)
        check(close(node.ins.ins_vel_n, 1.5), 'VX1000 north velocity 1.5', failures)
        check(close(node.ins.ins_heading, 90.0), 'VX1000 heading 90', failures)
        check(node.ins.navi_tick == 4, 'VX1000 navi_tick 4', failures)

    check(node.imu is not None, 'VX1000 /VX1000_imu received', failures)
    if node.imu:
        check(close(node.imu.gyr_x, 0.0, 1e-6), 'VX1000 gyr_x 0', failures)
        check(close(node.imu.acc_z, 9.8), 'VX1000 acc_z 9.8', failures)
    check(node.std_imu is not None, 'VX1000 /imu/data received', failures)
    if node.std_imu:
        check(close(node.std_imu.linear_acceleration.z, 9.8),
              '/imu/data linear_acceleration.z 9.8', failures)
        check(close(node.std_imu.orientation_covariance[0], -1.0),
              '/imu/data orientation unavailable', failures)

    check(node.gnss is not None, 'VX1000 /VX1000_gnss received', failures)
    if node.gnss:
        check(close(node.gnss.gnss_lat, 39.9), 'VX1000 GNSS latitude 39.9', failures)
        check(node.gnss.gnss_fix_mode == 3, 'VX1000 GNSS fix mode 3', failures)
        check(node.gnss.utc_year == 2026, 'VX1000 UTC year 2026', failures)
        check(node.gnss.utc_month == 10, 'VX1000 UTC month 10', failures)
        check(close(node.gnss.gnss_speed, 1.5), 'VX1000 GNSS speed 1.5', failures)
    check(node.std_fix is not None, 'VX1000 /gnss/fix received', failures)
    if node.std_fix:
        check(node.std_fix.status.status == NavSatStatus.STATUS_FIX,
              '/gnss/fix STATUS_FIX', failures)
        check(close(node.std_fix.latitude, 39.9), '/gnss/fix latitude 39.9', failures)

    check(node.stat is not None, 'VX1000 /VX1000_stat received', failures)
    check(node.cov is not None, 'VX1000 /VX1000_cov received', failures)
    check(node.odom is not None, 'VX1000 /VX1000_odom received', failures)
    if node.odom:
        check(close(node.odom.vehicle_spd, 1.5), 'VX1000 vehicle speed 1.5', failures)
        check(close(node.odom.steer_wheel_angle, 30.0), 'VX1000 steer 30', failures)

    os.close(master_fd)


def can_udp_frame(can_id, data):
    frame = bytearray([8, (can_id >> 24) & 0xFF, (can_id >> 16) & 0xFF,
                       (can_id >> 8) & 0xFF, can_id & 0xFF])
    frame.extend(data)
    return bytes(frame)


def make_vehicle_status():
    writer = BitWriter(8)
    writer.write(1, 2)              # gear D
    writer.write(0, 1)              # brake
    writer.write(1, 2)              # ignition
    writer.write(0, 2)              # reserved
    writer.write(1, 1)              # wheel direction: right
    writer.write(331, 12)           # 331 / 31.48 = 10.51 deg
    writer.write(15, 9)             # 1.5 m/s
    writer.write(1, 3)              # drive mode auto
    writer.write(1, 2)              # acc level
    writer.write(0, 2)              # brake level
    writer.write(12345, 20)         # odometer
    writer.write(0, 8)              # lights / horn
    assert writer.pos == 64
    return bytes(writer.buf)


def make_wheel(rpm):
    data = bytearray(8)
    struct.pack_into('<H', data, 0, 10)     # torque
    struct.pack_into('<h', data, 2, rpm)    # rpm
    struct.pack_into('<H', data, 4, 5)      # current
    return bytes(data)


def test_chassis(failures):
    chassis = subprocess.Popen(
        ['ros2', 'run', 'smartcar_chassis', 'chassis_node',
         '--ros-args', '-p', 'bind_address:=127.0.0.1',
         '-p', 'local_port:=18002'],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
        start_new_session=True)
    odom = subprocess.Popen(
        ['ros2', 'run', 'smartcar_chassis', 'can_odometry_node'],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
        start_new_session=True)

    node = Collector()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    frames = [
        can_udp_frame(0x51, make_vehicle_status()),
        can_udp_frame(0x178, make_wheel(100)),
        can_udp_frame(0x188, make_wheel(100)),
    ]

    def feed():
        for frame in frames:
            sock.sendto(frame, ('127.0.0.1', 18002))
        time.sleep(0.05)

    try:
        wait_until(node, lambda: node.status is not None, 8.0, feed)
        spin_for(node, 2.0, feed)
    finally:
        node.destroy_node()
        sock.close()
        outs = [stop(p) for p in (chassis, odom)]

    if node.status is None:
        print(outs[0][-2000:])
    check(node.status is not None, 'chassis /vehicle_status received', failures)
    if node.status:
        check(close(node.status.speed, 1.5), 'chassis speed 1.5 m/s', failures)
        check(close(node.status.steer, 10.51, 0.01), 'chassis steer 10.51 deg', failures)
        check(node.status.gear == 1, 'chassis gear D', failures)
        check(close(node.status.total_odometer, 12345), 'chassis odometer', failures)

    check(node.odo is not None, 'chassis /odom received', failures)
    if node.odo:
        expected = math.tan(-math.radians(10.51)) * 1.5 / 1.2
        check(close(node.odo.twist.twist.angular.z, expected, 1e-2),
              'odom angular.z matches Ackermann model', failures)
        check(node.odo.twist.twist.linear.x > 1.4, 'odom linear.x forward', failures)
        check(node.odo.pose.pose.position.x > 0.0, 'odom x advances', failures)


def main():
    rclpy.init()
    failures = []
    print('== VX1000 ==')
    test_vx(failures)
    print('== chassis ==')
    test_chassis(failures)
    rclpy.shutdown()

    if failures:
        print(f'\n{len(failures)} FAILURE(S)')
        return 1
    print('\nALL CHECKS PASSED')
    return 0


if __name__ == '__main__':
    sys.exit(main())
