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
        LogInfo(msg=['[smartcar:chassis] skeleton; config_dir=', config_dir]),
        Node(
            package='smartcar_bringup',
            executable='keepalive',
            name='smartcar_keepalive_chassis',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[{'role': 'chassis'}],
        ),
        # TODO: start the smartcar_chassis ROS 2 node here.
        # Gateway: 192.168.100.110, CAN1 UDP 4001, CAN2 UDP 4002,
        # local UDP 8002. Publish /vehicle_status and control messages.
    ])
