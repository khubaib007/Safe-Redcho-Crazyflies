from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    profile = LaunchConfiguration('config_file')
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=str(Path.home()/'safe_redcho_hardware/config/redcho_hardware.yaml')),
        *[Node(package='safe_redcho_flight', executable='redcho_agent', name=f'redcho_agent_{i}',
               output='screen', arguments=['--config', profile, '--agent', str(i)]) for i in range(1, 7)],
        Node(package='safe_redcho_flight', executable='redcho_watchdog', name='redcho_watchdog',
             output='screen', arguments=['--config', profile]),
    ])
