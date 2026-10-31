#!/usr/bin/env python3
from pathlib import Path
import sys

root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(root/'src/safe_redcho_bringup'))
from safe_redcho_bringup.configuration import prepare

lab, directory = prepare(root/'config/lab_config.yaml')
print('Generated configuration:', directory)
print('Motive server:', lab['network']['motive_ip'])
print('Expected rigid body:', lab['robot']['rigid_body_name'])
print('Radio URI:', lab['robot']['uri'])
if lab['robot']['uri'] == 'SET_YOUR_RADIO_URI':
    print('Pose-only launch is ready; set the actual radio URI before single_cf.launch.py')
