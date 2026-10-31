from datetime import datetime, timezone
from pathlib import Path
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def start(context):
    output = LaunchConfiguration('output').perform(context)
    if not output:
        config = yaml.safe_load(Path(LaunchConfiguration('config_file').perform(context)).expanduser().read_text())
        output = str(Path(config['output_root']).expanduser()/
                     datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ'))
    return [Node(package='safe_redcho_three_telemetry',executable=f'{key}_telemetry',output='screen',
                 condition=IfCondition(LaunchConfiguration(key)),
                 arguments=['--config',LaunchConfiguration('config_file'),'--output',output])
            for key in ('flight','reference','redcho','control')]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config_file',default_value=str(Path.home()/'safe_redcho_hardware/config/telemetry_three.yaml')),
        DeclareLaunchArgument('output',default_value=''),
        *[DeclareLaunchArgument(key,default_value='true') for key in ('flight','reference','redcho','control')],
        OpaqueFunction(function=start)])
