"""Reuse the working local Crazyswarm2/OptiTrack backend; no controller flash."""
from copy import deepcopy
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from safe_redcho_bringup.fleet_configuration import prepare_fleet
from safe_redcho_flight.settings import load


def start(context):
    cfg = load(LaunchConfiguration('config_file').perform(context))
    _, generated, robot_file = prepare_fleet(cfg['fleet_config_file'])
    robots = yaml.safe_load(Path(robot_file).read_text())
    robots['all']['firmware_logging']['default_topics']['status'] = {'frequency': 2}
    robots['all']['firmware_logging']['default_topics']['pose'] = {'frequency': 20}
    # Optional startup log blocks. No runtime radio reconfiguration by recorders.
    telemetry_file = Path(cfg['workspace'])/'config/telemetry_onboard.yaml'
    if telemetry_file.is_file():
        from safe_redcho_telemetry.onboard import apply_logging_profile
        apply_logging_profile(robots, yaml.safe_load(telemetry_file.read_text()))
    source = Path(get_package_share_directory('crazyflie'))/'config/server.yaml'
    server = deepcopy(yaml.safe_load(source.read_text())['/crazyflie_server']['ros__parameters'])
    mocap = yaml.safe_load((Path(generated)/'motion_capture.yaml').read_text())['/motion_capture_tracking']['ros__parameters']
    mocap['rigid_bodies'] = {}
    server.setdefault('firmware_params', {})['query_all_values_on_connect'] = True
    server['poses_qos_deadline'] = float(mocap['topics']['poses']['qos']['deadline'])
    description = Path(get_package_share_directory('crazyflie_description'))/'urdf/crazyflie_description.urdf'
    server['robot_description'] = description.read_text()
    get_package_share_directory('crazyflie_server_cpp')
    return [Node(package='motion_capture_tracking', executable='motion_capture_tracking_node',
                 name='motion_capture_tracking', parameters=[mocap], output='screen'),
            Node(package='crazyflie_server_cpp', executable='crazyflie_server',
                 name='crazyflie_server', parameters=[robots, server], output='screen')]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=str(Path.home()/'safe_redcho_hardware/config/redcho_hardware.yaml')),
        OpaqueFunction(function=start)])
