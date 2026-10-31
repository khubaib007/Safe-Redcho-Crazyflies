from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

GROUPS = ('flight','battery','timing','reference','velocity_redcho','position_redcho','control','hocbf')


def generate_launch_description():
    config = str(Path(get_package_share_directory('safe_redcho_three_liveplots'))/'config/liveplots.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config_file',default_value=config),
        DeclareLaunchArgument('capture',default_value='true',description='Disable only if live_capture already runs'),
        *[DeclareLaunchArgument(g,default_value='true') for g in GROUPS],
        Node(package='safe_redcho_three_liveplots',executable='live_capture',output='screen',
             arguments=['--config',LaunchConfiguration('config_file')],
             condition=IfCondition(LaunchConfiguration('capture'))),
        *[Node(package='safe_redcho_three_liveplots',executable=g+'_plot',output='screen',
               arguments=['--config',LaunchConfiguration('config_file')],
               condition=IfCondition(LaunchConfiguration(g))) for g in GROUPS],
    ])
