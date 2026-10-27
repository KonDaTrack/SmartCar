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
        LogInfo(msg=['[smartcar:slam] skeleton; config_dir=', config_dir]),
        Node(
            package='smartcar_bringup',
            executable='keepalive',
            name='smartcar_keepalive_slam',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[{'role': 'slam'}],
        ),
        # TODO: choose and wire the SLAM stack:
        #   FAST-LIO2 / Point-LIO / NDT localization / other.
        # Subscribe to /lslidar_point_cloud, /VX1000_imu, /tf, /vehicle_status.
    ])
