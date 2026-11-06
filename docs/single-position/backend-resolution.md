# Duplicate backend prevented ground readiness

The repeated `Require backend and at most one known control recorder on
cmd_position` message came from the command ownership guard. It requires exactly
one native backend command subscription per aircraft. This was unrelated to
REDCHO, HOCBF-QP feasibility or the tracking tube.

The old backend PID **91070**, started at 17:55 on 6 November 2026 (Asia/Riyadh),
survived its launcher's Ctrl+C. The backend for the reported check, PID **95157**,
started at 18:20 while that obsolete backend was still alive. Historical launch
logs and the subsequently inspected live PID/executable/launch parameter files
identify that overlap. The newer backend was stopped by the operator after the
failed check. Direct DDS inspection still found the old backend's three command
subscriptions.

After the user confirmed all three aircraft grounded and disarmed, PID 91070 was
rechecked against its executable and exact launch parameter files, then stopped
with one SIGINT. No other processes were signalled. Direct inspection verified
that all its command subscriptions disappeared. No flight commands were sent.

## Small patch on the restored 0.3.0 baseline

- Backend launch now probes the live DDS graph before starting mocap or radios.
  An existing backend, command publisher, unexpected command subscriber or
  `/poses` publisher prevents startup and produces a report in
  `results/single_backend_startup/`.
- Preflight retains the original endpoint policy, but now reports the aircraft
  topic, backend/recorder counts and unexpected endpoints. Its last failure is
  saved in `mission_status.json` as `last_route_fault`.
- The requested Ctrl+C supervisor remains active. REDCHO, guidance, QPs,
  tracking bounds, mission stages and all YAML profiles are unchanged from the
  restored 0.3.0 baseline. The package version remains 0.3.0.

The pre-change launcher, coordinator and command guide are backed up in
`before/`. `audit.json` compares the active source against the original release.

## Validation

- Rebuilt the dedicated `install_single_position` overlay; installed startup
  probe matches the final source byte for byte.
- **42 focused tests passed**, covering ownership counts, actual coordinator
  diagnostics with test doubles, landing and scoped Ctrl+C cleanup.
- **Actual DDS regression passed on isolated domain 181**, with separate harmless
  backend-shaped processes and explicit discovery/removal waits. It verified
  standalone probe initialization, duplicate rejection, one-backend acceptance
  by the preflight policy, refusal to start another backend, clear status after
  cleanup, and refusal while standalone mocap is present. No command messages
  were sent. Evidence: `inert_dds.json` and `inert_dds.log`.
- The final installed startup probe returned **CLEAR on hardware domain 43**
  after the obsolete process was stopped. It did not start radios. Evidence:
  `live_startup_probe_after.json` and `live_graph_after_cleanup.json`.

## Restart the single stack

The cleanup verification found no active mocap, agents or watchdog. Start these
commands in separate terminals from `~/safe_redcho_hardware`:

Terminal A:

```bash
bash scripts/redcho_single_mission.sh combined backend
```

Terminal B:

```bash
bash scripts/redcho_single_mission.sh combined agents
```

Optional terminal C, CSV telemetry with all live plots off:

```bash
bash scripts/redcho_single_mission.sh combined observe \
  recorders:=true capture:=false \
  flight:=false battery:=false timing:=false reference:=false \
  position_redcho:=false control:=false hocbf:=false
```

Terminal D:

```bash
bash scripts/redcho_single_mission.sh combined doctor
bash scripts/redcho_single_mission.sh combined check
```

Use the existing survey only if the source profile and aircraft/obstacle starts
still match. Otherwise stop the stack while grounded/disarmed, follow the
capture/preview sequence in `RUN_COMMANDS.md`, stop standalone mocap, then restart
one backend and the agents. These changes do not require recapture by themselves.

The complete physical ground check still needs the restarted backend, agents and
current measurements. It was not run automatically, and no flight was attempted.
