#!/usr/bin/env bash
set -e
task_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONNOUSERSITE=1
python3 - "$task_root/config/lab_config.yaml" <<'PY'
import json
from pathlib import Path
import subprocess
import sys
import yaml
lab = yaml.safe_load(Path(sys.argv[1]).read_text())
server = lab['network']['motive_ip']
local = lab['network']['ubuntu_ip']
print('Route to Motive:')
subprocess.run(['ip', 'route', 'get', server], check=True)
routes = json.loads(subprocess.check_output(['ip', '-j', 'route', 'get', server], text=True))
route = routes[0]
if route.get('prefsrc') != local:
    raise SystemExit(f'Wrong source IP: expected {local}, got {route.get("prefsrc")}')
print('Ethernet interface:', route['dev'])
print('Multicast route:')
subprocess.run(['ip', 'route', 'get', lab['network']['multicast_group']], check=True)
print('Ping (ICMP may be blocked even when NatNet works):')
subprocess.run(['ping', '-c', '3', '-W', '1', '-I', local, server], check=False)
print('Inspect actual NatNet packets, with the receiver running:')
print(f'sudo tcpdump -ni {route["dev"]} "udp and host {server} and (port 1510 or port 1511)"')
PY
