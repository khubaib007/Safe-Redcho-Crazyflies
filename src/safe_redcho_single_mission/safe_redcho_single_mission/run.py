"""Scenario launcher. Flight occurs only through explicit acquire/fly modes."""
import argparse
from datetime import datetime, timezone
from pathlib import Path
import sys

from .settings import load
from .node_shutdown import run as run_owned

SCENARIOS = ('default', 'combined', 'obstacle', 'interagent')
MODES = ('doctor', 'mocap', 'capture', 'survey', 'backend', 'agents', 'check',
         'offline', 'preview', 'observe', 'acquire', 'fly')


def command(workspace, scenario, mode, extra=()):
    """Resolve profiles before building commands; never invent live starts."""
    root = Path(workspace).expanduser().resolve()
    stem = 'single_mission' + ('' if scenario == 'default' else '_' + scenario)
    base = root / 'config' / (stem + '.yaml')
    surveyed = root / 'config/generated_single_hocbf' / (scenario + '.yaml')
    package = 'safe_redcho_single_mission'
    if mode == 'mocap':
        return ['ros2', 'launch', package, 'survey_mocap.launch.py', 'config_file:=' + str(base)]
    if mode in ('capture', 'survey'):
        return [sys.executable, '-s', '-m', package + '.survey', '--config', str(base), '--output', str(surveyed)]
    if mode in ('doctor', 'offline'):
        profile = base
        load(profile)
    else:
        from .survey import resolve_profile
        profile = resolve_profile(base, surveyed)
    output = root / 'results' / ('single_mission_' + scenario + '_' + mode + '_' +
                               datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ'))
    if mode in ('backend', 'agents'):
        return ['ros2', 'launch', package, mode + '.launch.py', 'config_file:=' + str(profile)]
    if mode == 'observe':
        from .observation_config import prepare
        telemetry, plots = prepare(profile, root / 'config/single_mission_telemetry.yaml',
            root / 'config/single_mission_liveplots.yaml', root / 'config/generated_single_observers')
        return ['ros2', 'launch', package, 'observe.launch.py',
                'config_file:=' + str(profile), 'telemetry_config:=' + str(telemetry),
                'plot_config:=' + str(plots), *extra]
    if mode == 'doctor':
        return ['ros2', 'run', package, 'mission_doctor', '--config', str(profile)]
    if mode in ('offline', 'preview'):
        args = [sys.executable, '-s', '-m', package + '.preview', '--config', str(profile), '--output', str(output)]
        if mode == 'preview':
            args.append('--captured')
        elif scenario != 'default':
            args += ['--starts', str(root / 'config' / (stem + '_fixture.yaml'))]
        return args
    args = ['ros2', 'run', package, 'mission_coordinator', '--config', str(profile), '--output', str(output)]
    if mode in ('acquire', 'fly'):
        args.append('--fly')
    if mode == 'acquire':
        args.append('--acquire-only')
    return args


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('scenario', choices=SCENARIOS)
    parser.add_argument('mode', choices=MODES)
    parser.add_argument('--workspace', default=str(Path.home() / 'safe_redcho_hardware'))
    args, extra = parser.parse_known_args()
    if extra and args.mode != 'observe':
        parser.error('Additional arguments are supported only for observe launch options')
    try:
        cmd = command(args.workspace, args.scenario, args.mode, extra)
    except (RuntimeError, ValueError, OSError) as exc:
        parser.exit(2, str(exc) + '\n')
    raise SystemExit(run_owned(cmd,args.mode,args.workspace))


if __name__ == '__main__':
    main()
