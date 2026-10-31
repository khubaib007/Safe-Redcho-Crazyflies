"""Tracking only, for capturing the new floor placement before radio bringup."""
from pathlib import Path
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from safe_redcho_bringup.configuration import prepare
from safe_redcho_flight.settings import load


def start(context):
    cfg = load(LaunchConfiguration('config_file').perform(context))
    _, generated = prepare(Path(cfg['workspace'])/'config/lab_config.yaml', require_radio=False)
    parameters = yaml.safe_load((Path(generated)/'motion_capture.yaml').read_text())['/motion_capture_tracking']['ros__parameters']
    parameters['rigid_bodies'] = {}
    return [Node(package='motion_capture_tracking', executable='motion_capture_tracking_node',
                 name='motion_capture_tracking', parameters=[parameters], output='screen')]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=str(Path.home()/'safe_redcho_hardware/config/redcho_hardware.yaml')),
        OpaqueFunction(function=start)])
