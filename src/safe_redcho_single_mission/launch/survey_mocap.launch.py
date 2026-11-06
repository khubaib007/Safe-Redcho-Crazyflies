"""Mocap only: old obstacle survey coordinates do not restrict data acquisition."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from safe_redcho_single_mission.survey import read_base
from safe_redcho_single_mission.backend import prepare

def start(context):
    c=read_base(LaunchConfiguration('config_file').perform(context))
    mc,_,_=prepare(c,radio=False)
    return [Node(package='motion_capture_tracking',executable='motion_capture_tracking_node',
                 name='motion_capture_tracking',parameters=[mc],output='screen')]

def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('config_file',default_value=str(
        Path.home()/'safe_redcho_hardware/config/single_mission_combined.yaml')),
        OpaqueFunction(function=start)])
