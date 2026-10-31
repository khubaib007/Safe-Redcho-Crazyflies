from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    profile = LaunchConfiguration('config_file')
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=str(Path.home()/'safe_redcho_hardware/config/redcho_three.yaml')),
        *[Node(package='safe_redcho_three', executable='redcho_agent', name=f'redcho_agent_{i}',
               output='screen', arguments=['--config', profile, '--agent', str(i)]) for i in (2, 3, 4)],
        Node(package='safe_redcho_three', executable='redcho_watchdog', name='redcho_watchdog',
             output='screen', arguments=['--config', profile]),
    ])
