import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    port = LaunchConfiguration('port')
    baudrate = LaunchConfiguration('baudrate')
    transport = LaunchConfiguration('transport')
    udp_bind_address = LaunchConfiguration('udp_bind_address')
    udp_port = LaunchConfiguration('udp_port')
    udp_rcvbuf = LaunchConfiguration('udp_rcvbuf')
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
            'transport',
            default_value=os.environ.get('SMARTCAR_VX1000_TRANSPORT', 'serial'),
            description="VX1000 transport: 'serial' or 'udp'",
        ),
        DeclareLaunchArgument(
            'udp_bind_address',
            default_value=os.environ.get('SMARTCAR_VX1000_UDP_BIND', '0.0.0.0'),
            description='UDP bind address for the VX1000 Ethernet output',
        ),
        DeclareLaunchArgument(
            'udp_port',
            default_value=os.environ.get('SMARTCAR_VX1000_UDP_PORT', '9000'),
            description='UDP destination port configured in the VX1000',
        ),
        DeclareLaunchArgument(
            'udp_rcvbuf',
            default_value=os.environ.get('SMARTCAR_VX1000_UDP_RCVBUF', '1048576'),
            description='UDP socket receive buffer in bytes',
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
                'transport': transport,
                'udp_bind_address': udp_bind_address,
                'udp_port': ParameterValue(udp_port, value_type=int),
                'udp_rcvbuf': ParameterValue(udp_rcvbuf, value_type=int),
                'frame_id': frame_id,
                'imu_frame_id': imu_frame_id,
                'gnss_frame_id': gnss_frame_id,
            }],
        ),
    ])
