from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from safe_redcho_three_mission.settings import load
from safe_redcho_three_mission.backend import prepare

def start(context):
    c=load(LaunchConfiguration('config_file').perform(context));mc,robots,server=prepare(c)
    return [Node(package='motion_capture_tracking',executable='motion_capture_tracking_node',name='motion_capture_tracking',parameters=[mc],output='screen'),Node(package='crazyflie_server_cpp',executable='crazyflie_server',name='crazyflie_server',parameters=[robots,server],output='screen')]
def generate_launch_description():return LaunchDescription([DeclareLaunchArgument('config_file',default_value=str(Path.home()/'safe_redcho_hardware/config/three_mission.yaml')),OpaqueFunction(function=start)])
