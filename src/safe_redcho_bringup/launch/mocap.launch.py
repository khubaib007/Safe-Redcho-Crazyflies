from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from safe_redcho_bringup.configuration import default_config, prepare


def start(context):
    _, generated = prepare(LaunchConfiguration('lab_config_file').perform(context))
    return [Node(package='motion_capture_tracking', executable='motion_capture_tracking_node',
                 name='motion_capture_tracking', output='screen',
                 parameters=[str(generated / 'motion_capture.yaml'), {'use_sim_time': False}])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('lab_config_file', default_value=default_config()),
        OpaqueFunction(function=start)])
