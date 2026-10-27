#!/usr/bin/env python3
"""Placeholder node for the initial SmartCar skeleton.

It keeps the container alive and reports which role/config directory is in use.
Replace it with real driver / SLAM launch includes as the project is implemented.
"""

import os

import rclpy
from rclpy.node import Node


class KeepAlive(Node):
    def __init__(self):
        super().__init__('smartcar_keepalive')
        self.declare_parameter('role', 'unknown')
        self.role = self.get_parameter('role').get_parameter_value().string_value
        self.config_dir = os.environ.get('SMARTCAR_CONFIG_DIR', '/config')

        self.get_logger().info(
            f'SmartCar skeleton alive: role={self.role} config_dir={self.config_dir}'
        )
        self.get_logger().info(
            'Drivers/SLAM are not wired yet. Replace keepalive with real launch includes.'
        )
        self.create_timer(30.0, self._tick)

    def _tick(self):
        self.get_logger().info(f'SmartCar skeleton alive: role={self.role}')


def main(args=None):
    rclpy.init(args=args)
    node = KeepAlive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
