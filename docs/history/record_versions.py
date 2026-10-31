#!/usr/bin/env python3
import json
from pathlib import Path
import subprocess
from datetime import datetime, timezone

root = Path(__file__).resolve().parents[1]


def run(*args):
    return subprocess.check_output(args, text=True).strip()


repos = {}
for name in ('crazyswarm2', 'motion_capture_tracking'):
    path = root/'src'/name
    repos[name] = {'url': run('git', '-C', str(path), 'remote', 'get-url', 'origin'),
                   'commit': run('git', '-C', str(path), 'rev-parse', 'HEAD'),
                   'submodules': run('git', '-C', str(path), 'submodule', 'status', '--recursive')}
record = {'recorded_utc': datetime.now(timezone.utc).isoformat(), 'repositories': repos}
(root/'config/dependency_versions.json').write_text(json.dumps(record, indent=2)+'\n')
print(json.dumps(record, indent=2))
