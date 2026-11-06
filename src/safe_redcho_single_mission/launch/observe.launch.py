from datetime import datetime,timezone
from pathlib import Path
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

GROUPS=('flight','battery','timing','reference','position_redcho','control','hocbf')
def start(context):
    telemetry=LaunchConfiguration('telemetry_config').perform(context);plots=LaunchConfiguration('plot_config').perform(context)
    from safe_redcho_single_mission.observation_config import prepare
    from safe_redcho_single_mission.settings import load
    profile=LaunchConfiguration('config_file').perform(context)
    telemetry,plots=map(str,prepare(profile,telemetry,plots,
        Path(load(profile)['workspace']).expanduser()/'config/generated_single_observers'))
    output=LaunchConfiguration('output').perform(context)
    if not output:
        cfg=yaml.safe_load(Path(telemetry).expanduser().read_text());output=str(Path(cfg['output_root']).expanduser()/datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ'))
    return [*[Node(package='safe_redcho_single_mission',executable=g+'_telemetry',arguments=['--config',telemetry,'--output',output],output='screen',condition=IfCondition(LaunchConfiguration('recorders'))) for g in ('flight','reference','redcho','control')],
        Node(package='safe_redcho_single_mission',executable='live_capture',arguments=['--config',plots],output='screen',condition=IfCondition(LaunchConfiguration('capture'))),
        *[Node(package='safe_redcho_single_mission',executable=g+'_plot',arguments=['--config',plots],output='screen',condition=IfCondition(LaunchConfiguration(g))) for g in GROUPS]]
def generate_launch_description():
    root=Path.home()/'safe_redcho_hardware/config'
    return LaunchDescription([DeclareLaunchArgument('config_file',default_value=str(root/'single_mission.yaml')),DeclareLaunchArgument('telemetry_config',default_value=str(root/'single_mission_telemetry.yaml')),DeclareLaunchArgument('plot_config',default_value=str(root/'single_mission_liveplots.yaml')),DeclareLaunchArgument('output',default_value=''),DeclareLaunchArgument('recorders',default_value='true'),DeclareLaunchArgument('capture',default_value='true'),
        *[DeclareLaunchArgument(g,default_value='true') for g in GROUPS],OpaqueFunction(function=start)])
