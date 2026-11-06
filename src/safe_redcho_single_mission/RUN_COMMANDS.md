# Single-position hardware commands

Use the [complete repository run guide](../../docs/single-position/START_HERE.md)
for setup, all tests, capture/preview, backend, agents, telemetry with plots off,
ground checks, acquisition and flight commands, and troubleshooting.

The published entry point preserves the familiar command spelling:

```bash
bash scripts/redcho_single_mission.sh combined init
bash scripts/redcho_single_mission.sh combined offline
```

Live operations read `config/local/` and require a current captured profile.
The [original workstation command record](../../docs/single-position/original-workspace-commands.md)
is historical; it assumes the original dedicated workspace/overlay layout.
