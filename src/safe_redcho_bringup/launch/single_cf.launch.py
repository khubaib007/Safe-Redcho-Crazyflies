import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from safe_redcho_bringup.configuration import default_config, prepare


def start(context):
    _, generated = prepare(LaunchConfiguration('lab_config_file').perform(context), True)
    source = os.path.join(get_package_share_directory('crazyflie'), 'launch', 'launch.py')
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(source),
            launch_arguments={
                'crazyflies_yaml_file': str(generated / 'crazyflies.yaml'),
                'motion_capture_yaml_file': str(generated / 'motion_capture.yaml'),
                'backend': 'cpp', 'mocap': 'True', 'gui': 'False',
                'rviz': 'False', 'teleop': 'False'}.items())]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('lab_config_file', default_value=default_config()),
        OpaqueFunction(function=start)])
