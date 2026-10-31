"""Hardware bringup with explicit firmware-value initialization.

Keeps upstream defaults, then enables query_all_values_on_connect last. Does
not modify third-party files or imply that startup caches stay current forever.
"""
from copy import deepcopy
from pathlib import Path
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from safe_redcho_bringup.fleet_configuration import default_fleet_config, prepare_fleet


def parameter_sets(generated, robots_file, upstream_share, description_share):
    generated, upstream_share = Path(generated), Path(upstream_share)
    with (upstream_share/'config/server.yaml').open() as handle:
        server = deepcopy(yaml.safe_load(handle)['/crazyflie_server']['ros__parameters'])
    with Path(robots_file).open() as handle:
        robots = yaml.safe_load(handle)
    with (generated/'motion_capture.yaml').open() as handle:
        mocap = yaml.safe_load(handle)['/motion_capture_tracking']['ros__parameters']
    mocap['rigid_bodies'] = {}  # Vendor tracking is solved by Motive.
    server.setdefault('firmware_params', {})['query_all_values_on_connect'] = True
    server['poses_qos_deadline'] = float(mocap['topics']['poses']['qos']['deadline'])
    server['robot_description'] = (
        Path(description_share)/'urdf/crazyflie_description.urdf').read_text()
    return mocap, [robots, server]


def start(context):
    _, generated, robots = prepare_fleet(
        LaunchConfiguration('fleet_config_file').perform(context))
    mocap_parameters, server_parameters = parameter_sets(
        generated, robots, get_package_share_directory('crazyflie'),
        get_package_share_directory('crazyflie_description'))
    # This workspace uses the split C++ package identified by the user.
    get_package_share_directory('crazyflie_server_cpp')
    return [
        Node(package='motion_capture_tracking', executable='motion_capture_tracking_node',
             name='motion_capture_tracking', output='screen', parameters=[mocap_parameters]),
        Node(package='crazyflie_server_cpp', executable='crazyflie_server',
             name='crazyflie_server', output='screen', parameters=server_parameters),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('fleet_config_file', default_value=default_fleet_config()),
        OpaqueFunction(function=start),
    ])
