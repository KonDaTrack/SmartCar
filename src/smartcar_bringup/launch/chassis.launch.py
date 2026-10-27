import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = LaunchConfiguration('config_dir')
    start_chassis = LaunchConfiguration('start_chassis')
    chassis_params = PathJoinSubstitution([config_dir, 'chassis.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_dir',
            default_value=os.environ.get('SMARTCAR_CONFIG_DIR', '/config'),
            description='Directory mounted with network/sensor/SLAM YAML',
        ),
        DeclareLaunchArgument(
            'start_chassis',
            default_value=os.environ.get('SMARTCAR_START_CHASSIS', '0'),
            description='Start the chassis gateway and odometry nodes',
        ),
        LogInfo(
            msg=['[smartcar:chassis] config_dir=', config_dir,
                 ' start_chassis=', start_chassis]),

        # Gateway receiver: 192.168.100.110 -> local UDP 8002.
        Node(
            package='smartcar_chassis',
            executable='chassis_node',
            name='chassis_node',
            output='screen',
            emulate_tty=True,
            condition=IfCondition(start_chassis),
            parameters=[chassis_params],
        ),

        # /vehicle_status -> /odom + TF odom -> base_footprint.
        Node(
            package='smartcar_chassis',
            executable='can_odometry_node',
            name='can_odometry_node',
            output='screen',
            emulate_tty=True,
            condition=IfCondition(start_chassis),
            parameters=[chassis_params],
        ),
    ])
