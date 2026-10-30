#!/usr/bin/env python3
"""Temporary test: create a bag with a known 50 ms wheel-IMU offset."""

import math
import shutil
import sys

from builtin_interfaces.msg import Time
from rclpy.serialization import serialize_message
from rosbag2_py import (
    ConverterOptions,
    SequentialWriter,
    StorageOptions,
    TopicMetadata,
)
from sensor_msgs.msg import Imu
from smartcar_msgs.msg import VehicleStatus

OUT = sys.argv[1]
RATE = 100.0
DURATION = 20.0
BASE = 1_700_000_000.0
DELAY = 0.05


def stamp(t):
    sec = int(t)
    return Time(sec=sec, nanosec=int((t - sec) * 1e9))


def yaw_signal(t):
    return 0.22 * math.sin(2.0 * math.pi * 0.2 * t)


shutil.rmtree(OUT, ignore_errors=True)
writer = SequentialWriter()
writer.open(
    StorageOptions(uri=OUT, storage_id='mcap'),
    ConverterOptions('', ''))
writer.create_topic(TopicMetadata(
    name='/vehicle_status',
    type='smartcar_msgs/msg/VehicleStatus',
    serialization_format='cdr'))
writer.create_topic(TopicMetadata(
    name='/imu/data',
    type='sensor_msgs/msg/Imu',
    serialization_format='cdr'))

for i in range(int(RATE * DURATION)):
    t = i / RATE

    wheel = VehicleStatus()
    wheel.header.stamp = stamp(BASE + t)
    wheel.header.frame_id = 'base_link'
    wheel.speed = 1.5
    wheel.gear = 1
    wheel.steer = math.degrees(math.atan(yaw_signal(t) * 1.2 / 1.5))
    writer.write('/vehicle_status', serialize_message(wheel),
                 int((BASE + t) * 1e9))

    imu = Imu()
    imu.header.stamp = stamp(BASE + t)
    imu.header.frame_id = 'vx1000_imu'
    # Same motion, delayed by 50 ms relative to the wheel timestamp.
    imu.angular_velocity.z = yaw_signal(t - DELAY)
    writer.write('/imu/data', serialize_message(imu),
                 int((BASE + t) * 1e9))

print(f'wrote {OUT}: wheel yaw at t, imu yaw at t-{DELAY}s')
