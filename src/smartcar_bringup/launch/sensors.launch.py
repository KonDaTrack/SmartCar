import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def _lakibeam(name, frame_id, output_topic, sensor_ip, port, host_ip):
    return Node(
        package='lakibeam1',
        executable='lakibeam1_scan_node',
        name=name,
        output='screen',
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
        LogInfo(msg=['[smartcar:sensors] config_dir=', config_dir, ' host_ip=', host_ip]),

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
                  '192.168.100.200', '2468', host_ip),
        _lakibeam('lakibeam_right', 'laser_right', 'scan_right',
                  '192.168.100.210', '2568', host_ip),
    ])
