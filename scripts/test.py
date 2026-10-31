"""Run only software/offline checks in isolated subprocesses; never live ROS commands."""
from datetime import datetime,timezone
from pathlib import Path
import json,subprocess,sys
root=Path(__file__).resolve().parents[1]
edition=json.loads((root/'release.json').read_text())['edition']
output=root/'results'/('tests_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ'));output.mkdir(parents=True)
commands=[['scripts/verify.py'],['-m','unittest','discover','-s','tests','-v']]
if edition=='three':commands.append(['-m','unittest','discover','-s','src/safe_redcho_three_liveplots/test','-v'])
if edition=='mission':commands.append(['-m','unittest','discover','-s','src/safe_redcho_three_mission/test','-v'])
commands.append(['scripts/run.py','offline','--output',str(output/'offline_default')])
if edition=='mission':
    commands.extend([['scripts/run.py','offline','--scenario',s,'--output',str(output/('offline_'+s))] for s in ['combined','interagent','obstacle']])
results=[]
for index,command in enumerate(commands):
    print('RUN',*command,flush=True)
    log=output/f'{index:02d}.log'
    with log.open('w') as handle:
        try:r=subprocess.run([sys.executable,*command],cwd=root,stdout=handle,stderr=subprocess.STDOUT,timeout=600);code=r.returncode
        except subprocess.TimeoutExpired:code=124
    results.append(dict(command=[s.replace(str(root),'$CHECKOUT') for s in command],exit_code=code,log=log.name))
    print('PASS' if code==0 else 'FAIL',command[:3],flush=True)
report=dict(status='PASS' if all(r['exit_code']==0 for r in results) else 'FAIL',edition=edition,checks=results,scope='Software, test-double and numerical checks only; no radio, live motion capture or aircraft commands')
(output/'test_report.json').write_text(json.dumps(report,indent=2)+'\n');print('Report:',output/'test_report.json')
raise SystemExit(0 if report['status']=='PASS' else 1)
