import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def _lakibeam(name, frame_id, output_topic, sensor_ip, port, host_ip, condition):
    return Node(
        package='lakibeam1',
        executable='lakibeam1_scan_node',
        name=name,
        output='screen',
        condition=condition,
        parameters=[{
            'frame_id': frame_id,
            'output_topic': output_topic,
            'inverted': False,
            'hostip': host_ip,
            'port': port,
            'angle_offset': 0,
            'sensorip': sensor_ip,
            'scanfreq': '30',
            'filter': '3',
            'laser_enable': 'true',
            'scan_range_start': '45',
            'scan_range_stop': '315',
        }],
    )


def _static_tf(name, parent, child, x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=name,
        output='screen',
        arguments=[
            '--x', str(x), '--y', str(y), '--z', str(z),
            '--roll', str(roll), '--pitch', str(pitch), '--yaw', str(yaw),
            '--frame-id', parent, '--child-frame-id', child,
        ],
    )


def generate_launch_description():
    config_dir = LaunchConfiguration('config_dir')
    host_ip = LaunchConfiguration('host_ip')
    lslidar_params = PathJoinSubstitution([config_dir, 'lslidar_cx.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_dir',
            default_value=os.environ.get('SMARTCAR_CONFIG_DIR', '/config'),
            description='Directory mounted with network/sensor/SLAM YAML',
        ),
        DeclareLaunchArgument(
            'host_ip',
            default_value=os.environ.get('SMARTCAR_HOST_IP', '192.168.100.123'),
            description='Host IP that the LakiBeam sensors send UDP scans to',
        ),
        DeclareLaunchArgument(
            'start_ins',
            default_value=os.environ.get('SMARTCAR_START_INS', '0'),
            description='Start the VX1000 serial driver (only on the vehicle)',
        ),
        DeclareLaunchArgument(
            'start_scan2d',
            default_value=os.environ.get('SMARTCAR_START_SCAN2D', '0'),
            description='Start the LakiBeam 2D scanners (not needed for LIO)',
        ),
        LogInfo(msg=['[smartcar:sensors] config_dir=', config_dir, ' host_ip=', host_ip]),

        # Static extrinsics from the active ROS 1 launch on this vehicle:
        #   docs/autox_startup.sh -> automotive_launch/autox_yuhuo_1.launch
        # Keep in sync with config/calibration/lidar_imu_extrinsics.yaml.
        _static_tf('base_to_rslidar', 'base_link', 'rslidar', 0.96, 0.0, 1.4),
        _static_tf('base_to_imu_link', 'base_link', 'imu_link', 0.0, 0.0, 0.2),
        _static_tf('base_to_gps', 'base_link', 'gps', 0.0, 0.0, 0.2),

        # VX1000 integrated navigation unit on /dev/ttyUSB0. Off by default so
        # the dev machine can run this launch file without the serial device.
        Node(
            package='smartcar_ins',
            executable='vx1000_node',
            name='vx1000_node',
            output='screen',
            emulate_tty=True,
            condition=IfCondition(LaunchConfiguration('start_ins')),
            parameters=[{
                'port': os.environ.get('SMARTCAR_VX1000_PORT', '/dev/ttyUSB0'),
                'baudrate': 460800,
                'frame_id': 'vx1000_link',
                'imu_frame_id': 'imu_link',
                'gnss_frame_id': 'gps',
            }],
        ),

        # LSLIDAR C16 (3D), ROS 2 CX driver. Namespace /cx matches the vendor
        # parameter file key. Remap the vendor topic to the project topic.
        Node(
            package='lslidar_driver',
            namespace='cx',
            executable='lslidar_driver_node',
            name='lslidar_driver_node',
            output='screen',
            emulate_tty=True,
            parameters=[lslidar_params],
            remappings=[('/cx/lslidar_point_cloud', '/lslidar_point_cloud')],
        ),

        # LakiBeam 2D left/right. The driver configures the sensor's destination
        # host/port, so host_ip must be the wired address of this machine.
        _lakibeam('lakibeam_left', 'laser_left', 'scan_left',
                  '192.168.100.200', '2468', host_ip,
                  IfCondition(LaunchConfiguration('start_scan2d'))),
        _lakibeam('lakibeam_right', 'laser_right', 'scan_right',
                  '192.168.100.210', '2568', host_ip,
                  IfCondition(LaunchConfiguration('start_scan2d'))),
    ])
