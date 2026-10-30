import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    port = LaunchConfiguration('port')
    baudrate = LaunchConfiguration('baudrate')
    frame_id = LaunchConfiguration('frame_id')
    imu_frame_id = LaunchConfiguration('imu_frame_id')
    gnss_frame_id = LaunchConfiguration('gnss_frame_id')

    return LaunchDescription([
        DeclareLaunchArgument(
            'port',
            default_value=os.environ.get('SMARTCAR_VX1000_PORT', '/dev/ttyUSB0'),
            description='Serial device of the VX1000 integrated navigation unit',
        ),
        DeclareLaunchArgument(
            'baudrate', default_value='460800',
            description='VX1000 serial baud rate',
        ),
        DeclareLaunchArgument(
            'frame_id', default_value='vx1000_link',
            description='Frame id for the raw VX1000 topics',
        ),
        DeclareLaunchArgument(
            'imu_frame_id', default_value='imu_link',
            description='Frame id for /imu/data (legacy base_link child)',
        ),
        DeclareLaunchArgument(
            'gnss_frame_id', default_value='gps',
            description='Frame id for /gnss/fix (legacy base_link child)',
        ),
        Node(
            package='smartcar_ins',
            executable='vx1000_node',
            name='vx1000_node',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'port': port,
                'baudrate': ParameterValue(baudrate, value_type=int),
                'frame_id': frame_id,
                'imu_frame_id': imu_frame_id,
                'gnss_frame_id': gnss_frame_id,
            }],
        ),
    ])
