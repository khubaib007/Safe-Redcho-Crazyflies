"""Bind every observer to the exact immutable profile used by the controllers."""
from pathlib import Path
import yaml
from .settings import load
from .protocol import fingerprint


def prepare(profile, telemetry_template, plot_template, output):
    profile = Path(profile).expanduser().resolve()
    cfg = load(profile)
    key = fingerprint(cfg)
    directory = Path(output).expanduser().resolve() / key
    directory.mkdir(parents=True, exist_ok=True)
    telemetry = yaml.safe_load(Path(telemetry_template).expanduser().read_text())
    plots = yaml.safe_load(Path(plot_template).expanduser().read_text())
    telemetry['algorithm_profile_file'] = str(profile)
    telemetry_path = directory / 'telemetry.yaml'
    plots.update(profile_file=str(profile), telemetry_config_file=str(telemetry_path))
    for path, data in ((telemetry_path, telemetry), (directory / 'liveplots.yaml', plots)):
        text = yaml.safe_dump(data, sort_keys=False)
        if not path.exists() or path.read_text() != text:
            temporary = path.with_suffix('.tmp')
            temporary.write_text(text)
            temporary.replace(path)
    return telemetry_path, directory / 'liveplots.yaml'
