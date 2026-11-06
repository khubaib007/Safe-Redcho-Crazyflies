"""Portable repository adapter; controller and scoped shutdown use original code."""
import argparse
from datetime import datetime, timezone
from pathlib import Path
import subprocess
import sys
from safe_redcho_single_mission.run import MODES, SCENARIOS
from safe_redcho_single_mission.node_shutdown import run as run_owned

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = 'safe_redcho_single_mission'


def command(scenario, mode, extra=(), output=None):
    if extra and mode != 'observe':
        raise ValueError('Additional launch arguments are supported only for observe')
    stem = 'single_mission'+('' if scenario == 'default' else '_'+scenario)
    base = ROOT/'config'/('' if mode in ('offline', 'doctor') else 'local')/(stem+'.yaml')
    if not base.is_file():
        raise ValueError('Initialize and edit lab settings first: bash scripts/redcho_single_mission.sh combined init')
    surveyed = ROOT/'config/generated_single_hocbf'/(scenario+'.yaml')
    if mode == 'mocap':
        return ['ros2', 'launch', PACKAGE, 'survey_mocap.launch.py', 'config_file:='+str(base)]
    if mode in ('capture', 'survey'):
        return [sys.executable, '-s', '-m', PACKAGE+'.survey', '--config', str(base), '--output', str(surveyed)]
    from safe_redcho_single_mission.settings import load
    if mode in ('offline', 'doctor'):
        profile = base
        load(profile)
    else:
        from safe_redcho_single_mission.survey import resolve_profile
        profile = resolve_profile(base, surveyed)
    output = output or ROOT/'results'/('single_mission_'+scenario+'_'+mode+'_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ'))
    if mode in ('backend', 'agents'):
        return ['ros2', 'launch', PACKAGE, mode+'.launch.py', 'config_file:='+str(profile)]
    if mode == 'observe':
        from safe_redcho_single_mission.observation_config import prepare
        telemetry, plots = prepare(profile, ROOT/'config/local/single_mission_telemetry.yaml',
                                   ROOT/'config/local/single_mission_liveplots.yaml', ROOT/'config/generated_single_observers')
        return ['ros2', 'launch', PACKAGE, 'observe.launch.py', 'config_file:='+str(profile),
                'telemetry_config:='+str(telemetry), 'plot_config:='+str(plots), *extra]
    if mode == 'doctor':
        return ['ros2', 'run', PACKAGE, 'mission_doctor', '--config', str(profile)]
    if mode in ('offline', 'preview'):
        args = [sys.executable, '-s', '-m', PACKAGE+'.preview', '--config', str(profile), '--output', str(output)]
        if mode == 'preview':
            args.append('--captured')
        elif scenario != 'default':
            args += ['--starts', str(ROOT/'config'/(stem+'_fixture.yaml'))]
        return args
    args = ['ros2', 'run', PACKAGE, 'mission_coordinator', '--config', str(profile), '--output', str(output)]
    if mode in ('acquire', 'fly'):
        args.append('--fly')
    if mode == 'acquire':
        args.append('--acquire-only')
    return args


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('scenario', choices=SCENARIOS)
    parser.add_argument('mode', choices=(*MODES, 'init', 'help'))
    parser.add_argument('--output', type=Path)
    args, extra = parser.parse_known_args()
    if args.mode == 'help':
        parser.print_help()
        print('Offline: init | offline. Hardware: mocap | capture | preview | backend | agents | observe | doctor | check | acquire | fly.')
        return 0
    if args.mode == 'init':
        if extra:
            parser.error('Unexpected init arguments')
        return subprocess.call([sys.executable, str(ROOT/'scripts/configure.py')])
    try:
        cmd = command(args.scenario, args.mode, extra, args.output)
    except (ValueError, RuntimeError, OSError) as exc:
        parser.exit(2, str(exc)+'\n')
    return run_owned(cmd, args.mode, ROOT)


if __name__ == '__main__':
    raise SystemExit(main())
