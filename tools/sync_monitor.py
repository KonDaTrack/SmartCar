#!/usr/bin/env python3
"""Measure arrival/stamp latency for the three LIO sensors.

Topics:
  /lslidar_point_cloud  sensor_msgs/PointCloud2
  /imu/data             sensor_msgs/Imu
  /vehicle_status       smartcar_msgs/VehicleStatus

The CSV is for diagnostics and later motion-based calibration. This node does
NOT rewrite timestamps and does NOT estimate the physical sensor offset by
itself: all three drivers currently stamp with the host clock at receive/parse
time, so the residual difference must be found from motion (see
tools/estimate_offsets.py and the LIO time_offset_lidar_to_imu parameter).

Environment:
  SYNC_LOG_DIR   output directory (default: /bags/sync)
"""

import csv
import os
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2
from smartcar_msgs.msg import VehicleStatus


class TopicStats:
    def __init__(self):
        self.count = 0
        self.latencies_ms = []
        self.last_stamp_ns = None
        self.max_gap_s = 0.0
        self.window_count = 0
        self.window_start = time.monotonic()

    def update(self, stamp_ns, arrival_ns):
        self.count += 1
        self.window_count += 1

        if stamp_ns:
            latency_ms = (arrival_ns - stamp_ns) / 1e6
            self.latencies_ms.append(latency_ms)
            if len(self.latencies_ms) > 5000:
                del self.latencies_ms[:2500]

            if self.last_stamp_ns is not None:
                gap_s = (stamp_ns - self.last_stamp_ns) / 1e9
                if 0.0 <= gap_s <= 2.0:
                    self.max_gap_s = max(self.max_gap_s, gap_s)
            self.last_stamp_ns = stamp_ns

    def summary(self):
        now = time.monotonic()
        elapsed = max(now - self.window_start, 1e-6)
        rate = self.window_count / elapsed
        self.window_count = 0
        self.window_start = now

        if not self.latencies_ms:
            return f"{rate:7.1f} Hz  no valid stamp"

        values = sorted(self.latencies_ms)
        mean = sum(values) / len(values)
        p95 = values[min(len(values) - 1, int(len(values) * 0.95))]
        return (
            f"{rate:7.1f} Hz  latency ms: mean={mean:7.2f} "
            f"p95={p95:7.2f} max={values[-1]:7.2f}  "
            f"max_gap={self.max_gap_s * 1000:6.1f} ms")


class OffsetMonitor(Node):
    TOPICS = (
        ('lidar', '/lslidar_point_cloud', PointCloud2),
        ('imu', '/imu/data', Imu),
        ('wheel', '/vehicle_status', VehicleStatus),
    )

    def __init__(self):
        super().__init__('time_offset_monitor')
        default_dir = os.environ.get('SYNC_LOG_DIR', '/bags/sync')
        self.output_dir = self.declare_parameter('output_dir', default_dir).value
        self.log_period_s = float(self.declare_parameter('log_period_s', 5.0).value)

        self.csv_file = None
        self.csv_writer = None
        self.csv_path = None
        try:
            os.makedirs(self.output_dir, exist_ok=True)
            stamp = time.strftime('%Y%m%d_%H%M%S', time.localtime())
            self.csv_path = os.path.join(self.output_dir, f'sync_{stamp}.csv')
            self.csv_file = open(self.csv_path, 'w', newline='', encoding='utf-8')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(
                ['wall_ns', 'topic', 'stamp_ns', 'latency_ms', 'frame_id'])
        except OSError as exc:
            self.get_logger().warning(f'cannot write sync CSV: {exc}')

        self.stats = {}
        for name, topic, msg_type in self.TOPICS:
            self.stats[name] = TopicStats()
            self.create_subscription(
                msg_type, topic,
                lambda msg, key=name: self.on_message(key, msg),
                qos_profile_sensor_data)

        self.timer = self.create_timer(self.log_period_s, self.log_summary)
        self.get_logger().info(
            f'time offset monitor started; csv={self.csv_path}; '
            f'topics={[t for _, t, _ in self.TOPICS]}')
        self.get_logger().info(
            'latency = arrival - header.stamp; physical offsets require motion calibration')

    def on_message(self, name, msg):
        arrival_ns = self.get_clock().now().nanoseconds
        stamp_ns = (msg.header.stamp.sec * 1_000_000_000 +
                    msg.header.stamp.nanosec)
        self.stats[name].update(stamp_ns, arrival_ns)

        if self.csv_writer is not None:
            latency_ms = ((arrival_ns - stamp_ns) / 1e6) if stamp_ns else ''
            self.csv_writer.writerow([
                arrival_ns, name, stamp_ns, latency_ms, msg.header.frame_id])

    def log_summary(self):
        parts = []
        for name, _, _ in self.TOPICS:
            parts.append(f'{name}: {self.stats[name].summary()}')
        self.get_logger().info(' | '.join(parts))
        if self.csv_file is not None:
            self.csv_file.flush()

    def destroy_node(self):
        if self.csv_file is not None:
            self.csv_file.flush()
            self.csv_file.close()
            self.csv_file = None
        super().destroy_node()


def main():
    rclpy.init()
    node = OffsetMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
