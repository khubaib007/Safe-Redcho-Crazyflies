"""Create local editable configuration from examples; never capture/fabricate live poses."""
from pathlib import Path
import json,yaml
root=Path(__file__).resolve().parents[1]
local=root/'config/local';local.mkdir(exist_ok=True)
def translate(value):
    if isinstance(value,dict):return {k:(str(root) if k=='workspace' else translate(v)) for k,v in value.items()}
    if isinstance(value,list):return [translate(v) for v in value]
    if isinstance(value,str):
        if value.startswith('config/'):return str(root/'config/local'/Path(value).name)
        if value.startswith('results/'):return str(root/value)
    return value
for p in (root/'config').glob('*.yaml'):
    if p.name in ['dependency_versions.yaml']:continue
    target=local/p.name
    if target.exists():continue
    target.write_text(yaml.safe_dump(translate(yaml.safe_load(p.read_text())),sort_keys=False))
print('Local configuration created:',local)
print('Edit radio URIs, network, measured arena and aircraft settings before hardware bringup. Capture starts on the actual floor; no captured-start file was created.')
