# Software validation — 2026-11-02

**Historical pre-enhancement baseline record.** The current version 0.2.0 adds avoidance features and different profile parameters. See [ROBUST_AVOIDANCE.md](ROBUST_AVOIDANCE.md) and `results/single_avoidance_final_20261102/` for current validation; the 146-test count and parameter-matching statements below refer to the frozen earlier baseline.

Implemented `safe_redcho_single_mission` as an isolated, single-position-bank baseline. Each of cf2/cf3/cf4 owns one order-2 position REDCHO bank and one local reciprocal HOCBF-QP. The reference generator and numerical control/safety parameters match the corresponding existing mission profiles; only the unused velocity-bank parameter is removed. Position-bank derivative outputs replace the separate horizontal velocity-bank estimates.

All validation below is software or synthetic-data validation. **No aircraft command, backend/radio client, physical takeoff, flight, landing or disarm was started.** Previous dual-bank flight results are not qualification evidence for this baseline.

## Checks completed

- ROS 2 Jazzy package build completed in separate `build_single_position`, `install_single_position`, `log_single_position` trees.
- Installed agent/coordinator/watchdog modules and actual Crazyflie/mocap ROS interfaces imported successfully.
- **146 tests passed, 2 skipped** in 13.15 seconds. Skips are inherited tests for a legacy ZIP installer not used here. Tests cover the single-bank derivative source across all seven streamed stages, position-only peers, exact-cycle adapters, rejected stale data, hard-QP/priority rules, survey tampering, observers, rendering, landing handover, journal-close failure and command-mode flight flags.
- All seven plot windows rendered offscreen, including position/velocity/acceleration and convergence panels from the position bank.
- Five installed launch descriptions loaded without executing launch actions. All four scenario observer profiles produced matching controller/recorder/plot configuration hashes.
- The runner's `combined doctor` completed `PASS_LOCAL_SOFTWARE`; this checks interface availability and local configuration, not live radio/firmware measurements.
- **257 existing source/config/script files checked against their pre-change SHA-256 hashes: zero changed.** Numeric scenario settings match their originals after excluding architecture, namespace, starts path and unused velocity-bank bound.

## Complete numerical mission previews

Nominal inputs were [4.0, 3.8, 3.7] V; unequal inputs were [3.55, 3.45, 3.35] V, ordered cf2/cf3/cf4. All cases completed WARMUP, HEIGHT, YAW, FORMATION, SETTLE, FORWARD and STOP. These use a synthetic first-order tracking model, not measured flight dynamics.

| Scenario | Battery | Result | Logical duration (s) | Minimum virtual XY separation (m) |
|---|---|---|---:|---:|
| Default | Nominal | PASS | 49.60 | 0.919872 |
| Default | Unequal | PASS | 63.06 | 0.919801 |
| Combined | Nominal | PASS | 82.56 | 0.722131 |
| Combined | Unequal | PASS | 114.86 | 0.722135 |
| Obstacle | Nominal | PASS | 47.72 | 0.788617 |
| Obstacle | Unequal | PASS | 61.18 | 0.788617 |
| Interagent | Nominal | PASS | 47.56 | 0.722175 |
| Interagent | Unequal | PASS | 61.02 | 0.722145 |

The combined nominal case used the permitted ordinary-slew relaxation on one agent sample: maximum slew 0.81538 m/s³, below the configured hard safety cap of 8.0 m/s³. Its safety rows, acceleration limits and speed limits were not softened. Other cases stayed within ordinary slew 0.8 m/s³. All final synthetic endpoint errors were below 0.00059 m; these values describe the specified numerical model only.

## Actual ROS process/transport check

Three installed agent processes exchanged synthetic WARMUP frames over real DDS on isolated ROS domain 173. The probe has no flight coordinator, aircraft command publisher, backend, watchdog or flight service client. Both `flight_active` and `streaming` were false.

- 250 complete same-cycle three-agent results accepted and safety-validated.
- 750 single-bank diagnostic packets received: 250 per aircraft.
- Three distinct process IDs observed.
- Median complete cycle 4.27 ms; maximum 15.60 ms, below the configured 80 ms deadline in this finite run.
- All spawned agent processes were stopped afterward.

The sandbox initially prevented ROS logging/discovery. Logs were redirected to the new validation directory, and the DDS check passed outside the restricted network sandbox. This finite transport result is not a hard real-time guarantee or a test of onboard delivery.

To reproduce the transport check after building, source the Jazzy, existing hardware and baseline overlays in a clean terminal, choose a separate non-production ROS domain, and run:

```bash
cd ~/safe_redcho_hardware
export PYTHONNOUSERSITE=1 ROS_DOMAIN_ID=173
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export ROS_LOG_DIR="$PWD/results/single_position_transport_manual/ros_log"
python3 -s src/safe_redcho_single_mission/test/inert_transport_check.py \
  --config config/single_mission_combined.yaml \
  --starts config/single_mission_combined_fixture.yaml \
  --output results/single_position_transport_manual --cycles 250
```

## Evidence

Workspace directory: `results/single_position_validation_20261102/`.

- `tests.log`, `tests.xml`, `build.log`, `doctor.json`, `launch_checks.json`.
- `summary.json` and eight scenario directories with effective configuration, plan, HOCBF row CSV and numerical trace.
- `inert_transport/report.json` and per-agent logs.
- `preservation.json`, `original_files_sha256.json`, and source hashes for this baseline.

Hardware commissioning remains: new ground survey, captured-plan preview, live ground-only check, acquisition-only flight and finally the full mission under operator supervision. No claim of single-bank superiority or physical validation follows from these software results.
