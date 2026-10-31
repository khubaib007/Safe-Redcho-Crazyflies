"""Fetch exact upstream/submodule commits, apply checked patches; no ROS/radio I/O."""
import argparse,hashlib,json,subprocess
from pathlib import Path
root=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser(description=__doc__);p.add_argument('--check',action='store_true');args=p.parse_args()
lock=json.loads((root/'dependency-lock.json').read_text())['repositories']
def git(path,*args):return subprocess.check_output(['git','-C',str(path),*args],text=True).strip()
for entry in [e for e in lock if e['relative_path']=='.']:
    dest=root/'src'/entry['repository']
    if not dest.exists():
        if args.check:raise SystemExit('Missing dependency: '+str(dest)+'; run python3 scripts/fetch_dependencies.py')
        subprocess.run(['git','clone','--no-checkout',entry['url'],str(dest)],check=True)
        subprocess.run(['git','-C',str(dest),'checkout','--detach',entry['commit']],check=True)
        subprocess.run(['git','-C',str(dest),'submodule','update','--init','--recursive'],check=True)
    if git(dest,'rev-parse','HEAD')!=entry['commit']:raise SystemExit('Wrong dependency commit: '+str(dest))
for entry in lock:
    dest=root/'src'/entry['repository']/entry['relative_path']
    if git(dest,'rev-parse','HEAD')!=entry['commit']:raise SystemExit('Wrong submodule commit: '+str(dest))
    if entry.get('patch'):
        patch=root/'patches/dependencies'/entry['patch']
        if hashlib.sha256(patch.read_bytes()).hexdigest()!=entry['patch_sha256']:raise SystemExit('Patch hash changed')
        if not args.check:
            ready=subprocess.run(['git','-C',str(dest),'apply','--check',str(patch)],capture_output=True).returncode==0
            if ready:subprocess.run(['git','-C',str(dest),'apply',str(patch)],check=True)
        for name,digest in entry['patched_files_sha256'].items():
            if not (dest/name).is_file() or hashlib.sha256((dest/name).read_bytes()).hexdigest()!=digest:raise SystemExit('Missing/changed patched dependency file: '+str(dest/name))
print('PASS: exact upstream/submodule revisions and required patches')
