from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    c=LaunchConfiguration('config_file')
    return LaunchDescription([DeclareLaunchArgument('config_file',default_value=str(Path.home()/'safe_redcho_hardware/config/single_mission.yaml')),
        *[Node(package='safe_redcho_single_mission',executable='mission_agent',arguments=['--config',c,'--agent',n],output='screen') for n in ('cf2','cf3','cf4')],Node(package='safe_redcho_single_mission',executable='mission_watchdog',arguments=['--config',c],output='screen')])
