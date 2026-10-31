"""Bind observers to exactly the executed profile; no ROS or hardware access."""
from pathlib import Path
import yaml
from .settings import load
from .protocol import fingerprint

def prepare(profile, telemetry, plots, output):
    profile=Path(profile).resolve();c=load(profile)
    telemetry=yaml.safe_load(Path(telemetry).read_text());plots=yaml.safe_load(Path(plots).read_text())
    directory=Path(output)/fingerprint(c);directory.mkdir(parents=True,exist_ok=True)
    telemetry['algorithm_profile_file']=str(profile)
    tpath=directory/'telemetry.yaml';ppath=directory/'plots.yaml'
    plots['profile_file']=str(profile);plots['telemetry_config_file']=str(tpath)
    tpath.write_text(yaml.safe_dump(telemetry,sort_keys=False));ppath.write_text(yaml.safe_dump(plots,sort_keys=False))
    return tpath,ppath
