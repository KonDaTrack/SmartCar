import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = LaunchConfiguration('config_dir')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_dir',
            default_value=os.environ.get('SMARTCAR_CONFIG_DIR', '/config'),
            description='Directory mounted with network/sensor/SLAM YAML',
        ),
        LogInfo(msg=['[smartcar:sensors] skeleton; config_dir=', config_dir]),
        Node(
            package='smartcar_bringup',
            executable='keepalive',
            name='smartcar_keepalive_sensors',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[{'role': 'sensors'}],
        ),
        # TODO: include the real drivers:
        #   - LSLIDAR C16 (192.168.100.201, UDP 2368/2369)
        #   - LakiBeam left  (192.168.100.200, UDP 2468)
        #   - LakiBeam right (192.168.100.210, UDP 2568)
        #   - VX1000         (/dev/ttyUSB0, 460800)
        # Keep IPs/ports in config/network.yaml and site YAML overrides.
    ])
