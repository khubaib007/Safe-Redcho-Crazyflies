"""Static/software verification only: no ROS initialization or hardware connection."""
import ast,json,subprocess,sys
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy,scipy,yaml
root=Path(__file__).resolve().parents[1]
for package in (root/'src').glob('safe_redcho*'):
    for p in package.rglob('*.py'):ast.parse(p.read_text(),filename=str(p))
    ET.parse(package/'package.xml')
for p in (root/'scripts').glob('*.sh'):subprocess.run(['bash','-n',str(p)],check=True)
print(json.dumps(dict(status='PASS_LOCAL_SOFTWARE',edition=json.loads((root/'release.json').read_text())['edition'],numpy=numpy.__version__,scipy=scipy.__version__,scope='Syntax/imports only; no hardware or flight validation'),indent=2))
