import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    config_dir = LaunchConfiguration('config_dir')
    share_dir = get_package_share_directory('smartcar_bringup')
    launch_dir = os.path.join(share_dir, 'launch')

    includes = []
    for name in ('sensors', 'chassis', 'slam'):
        includes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, f'{name}.launch.py')
                ),
                launch_arguments={'config_dir': config_dir}.items(),
            )
        )

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_dir',
            default_value=os.environ.get('SMARTCAR_CONFIG_DIR', '/config'),
            description='Directory mounted with network/sensor/SLAM YAML',
        ),
        LogInfo(msg=['[smartcar:vehicle] combined skeleton; config_dir=', config_dir]),
        *includes,
    ])
