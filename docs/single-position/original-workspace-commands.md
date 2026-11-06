This archived record assumes the original workstation. For this release use [START_HERE.md](START_HERE.md). Historical results/checkpoints named below are not bundled.

# Historical original-workspace command record

These commands select the `combined` scenario. Run only one aircraft controller stack at a time. The launcher sources the hardware environment automatically. Use clean terminals for hardware operations after the isolated ROS tests below.

The controller/profiles are restored to **0.3.0**. The separately requested
Ctrl+C cleanup is retained: script-launched commands immediately interrupt their
owned child process group, then stop hung non-flight children with short bounded
escalation. `fly` and `acquire` keep the original landing/recovery routine alive
until it exits; keep backend and agents/watchdog running through that recovery.
Backend startup also refuses an existing backend or `/poses` publisher before
opening radios; preflight failures now name the conflicting topic/endpoints.
See [rollback details](../../results/single_v030_rollback_20261106/ROLLBACK.md).

## 1. Build the separate overlay

```bash
cd ~/safe_redcho_hardware
set +u
export PYTHONNOUSERSITE=1
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
/usr/bin/colcon --log-base log_single_position build \
  --base-paths src \
  --build-base build_single_position \
  --install-base install_single_position \
  --packages-select safe_redcho_single_mission
source install_single_position/local_setup.bash
export PYTHONPATH="$PWD/src/safe_redcho_single_mission${PYTHONPATH:+:$PYTHONPATH}"
```

The offline commands below can use this terminal. No existing overlay is deleted.

## 2. Full software test suite

```bash
PYTHONNOUSERSITE=1 QT_QPA_PLATFORM=offscreen \
  /usr/bin/python3 -s -m pytest src/safe_redcho_single_mission/test \
  -q -p no:cacheprovider
```

This includes the focused tests below. GUI tests use an offscreen platform and open no desktop windows. Version 0.3.0 validation recorded 217 passing tests and 2 skipped legacy installer-format tests; subsequent source/configuration changes require their own checks.

## 3. Focused offline CBF-QP, predictive constraints and routing tests

Optional after the full suite; useful when diagnosing a specific subsystem.

```bash
PYTHONNOUSERSITE=1 /usr/bin/python3 -s -m pytest -q \
  src/safe_redcho_single_mission/test/test_obstacles.py \
  src/safe_redcho_single_mission/test/test_predictive_barriers.py \
  src/safe_redcho_single_mission/test/test_qp_priority.py \
  src/safe_redcho_single_mission/test/test_velocity_viability.py \
  src/safe_redcho_single_mission/test/test_responsibility.py \
  src/safe_redcho_single_mission/test/test_passing_encounters.py \
  src/safe_redcho_single_mission/test/test_route_guidance.py
```

These exercise the actual local QPs and complete batch checks, including invalid-state rejection, swaps, crossings, obstacles, near-wall routing and recorded full-mission regressions. They do not send aircraft commands.

## 4. Offline telemetry, adapter and measurement tests

```bash
PYTHONNOUSERSITE=1 /usr/bin/python3 -s -m pytest -q \
  src/safe_redcho_single_mission/test/test_ros_adapters.py \
  src/safe_redcho_single_mission/test/test_plot_data.py \
  src/safe_redcho_single_mission/test/test_state_receiver.py \
  src/safe_redcho_single_mission/test/test_single_position.py
```

These check packets, same-cycle diagnostics, convergence calculations, timestamp handling, stale telemetry, profile binding and recording-related behavior. Adapter tests use test doubles; the real-DDS check is separate below. No GUI windows are started by this command.

## 5. Complete numerical missions, with real HOCBF-QP filtering

```bash
bash scripts/redcho_single_mission.sh combined offline
```

Optional other fixtures:

```bash
bash scripts/redcho_single_mission.sh default offline
bash scripts/redcho_single_mission.sh obstacle offline
bash scripts/redcho_single_mission.sh interagent offline
```

`offline` already runs reference generation, the single REDCHO bank, route guidance, per-agent HOCBF-QPs, complete hard-constraint checks and a synthetic tracker. There is no separate flag needed to enable the CBF-QP. It writes the report, trace and HOCBF row evidence to a timestamped `results/single_mission_<scenario>_offline_*` directory.

For unequal synthetic voltages:

```bash
PYTHONNOUSERSITE=1 /usr/bin/python3 -s -m safe_redcho_single_mission.preview \
  --config config/single_mission_combined.yaml \
  --starts config/single_mission_combined_fixture.yaml \
  --voltages 3.55 3.45 3.35 \
  --output "results/single_combined_unequal_$(date -u +%Y%m%dT%H%M%SZ)"
```

Optional complete ten-case matrix, including the 7.5 s formation-phase stress request:

```bash
PYTHONNOUSERSITE=1 /usr/bin/python3 -s \
  src/safe_redcho_single_mission/test/validate_scenarios.py \
  --workspace "$PWD" \
  --output "results/single_matrix_$(date -u +%Y%m%dT%H%M%SZ)"
```

These numerical tests do not require a live mocap system, backend, aircraft or GUI. Synthetic starts do not replace a physical ground survey.

## 6. Real ROS/DDS agent and telemetry test, without aircraft

This uses three installed agent processes on isolated **domain 174**. Do not start the hardware backend, normal agents launcher or a flight coordinator for this test. The harness starts and cleans up its own three agents.

Run this preparation in both test terminals:

```bash
cd ~/safe_redcho_hardware
set +u
export PYTHONNOUSERSITE=1
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
source install_single_position/local_setup.bash
export ROS_DOMAIN_ID=174
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/config/fastdds_udp_only.xml"
export FASTDDS_DEFAULT_PROFILES_FILE="$FASTRTPS_DEFAULT_PROFILES_FILE"
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT
unset ROS_LOCALHOST_ONLY ROS_STATIC_PEERS
```

Test terminal 1: start the actual four CSV recorders, with all plot windows and extra live-plot event capture off:

```bash
ros2 launch safe_redcho_single_mission observe.launch.py \
  config_file:="$PWD/config/single_mission_combined.yaml" \
  output:="$PWD/results/single_telemetry_test_$(date -u +%Y%m%dT%H%M%SZ)" \
  recorders:=true capture:=false \
  flight:=false battery:=false timing:=false reference:=false \
  position_redcho:=false control:=false hocbf:=false
```

This direct observer launch intentionally binds the same synthetic source profile as the harness; it does not use a live captured profile. Keep this terminal running during the next command.

Test terminal 2: quick 250-cycle WARMUP/transport/telemetry test:

```bash
/usr/bin/python3 -s src/safe_redcho_single_mission/test/inert_transport_check.py \
  --config config/single_mission_combined.yaml \
  --starts config/single_mission_combined_fixture.yaml \
  --cycles 250 \
  --output "results/single_dds_quick_$(date -u +%Y%m%dT%H%M%SZ)"
```

Alternatively, exercise the complete synthetic mission and route/QP/telemetry pipeline:

```bash
/usr/bin/python3 -s src/safe_redcho_single_mission/test/inert_transport_check.py \
  --config config/single_mission_combined.yaml \
  --starts config/single_mission_combined_fixture.yaml \
  --full-mission --cycles 10000 \
  --output "results/single_dds_full_$(date -u +%Y%m%dT%H%M%SZ)"
```

The harness report checks three distinct agent processes, complete same-cycle commands, single-bank telemetry and deadlines. Full mode additionally checks synthetic stage completion and reports route exercise/timing. It creates no aircraft command publishers, radios or flight services. No physical `/poses`, `/cf*/pose`, battery or position-command stream is fabricated, so the isolated flight recorder will correctly report those streams as missing; algorithm/reference/control/convergence records should arrive.

After the harness finishes, stop test terminal 1 with Ctrl+C so writers flush. In that terminal, inspect the newest CSV test run:

```bash
telemetry_test_run=$(ls -td results/single_telemetry_test_*/ | head -n 1)
rg --files "$telemetry_test_run" \
  -g '*.csv' -g 'recorder_status.json' -g 'writer_summary.json'
cat "$telemetry_test_run"/*/writer_summary.json
```

Inspect `written_rows`, `dropped_batches`, `errors` and writer shutdown state. The harness's telemetry counts prove reception at the harness; the CSV writer summaries separately check the recording processes. Close these isolated test terminals before using fresh hardware terminals.

## 7. Hardware survey and captured numerical preview

Aircraft remain grounded. Stop other aircraft controllers before starting this sequence. In every new terminal begin with:

```bash
cd ~/safe_redcho_hardware
```

Terminal A: standalone mocap:

```bash
bash scripts/redcho_single_mission.sh combined mocap
```

Terminal B: capture stationary aircraft/obstacle positions, then preview that exact immutable profile:

```bash
bash scripts/redcho_single_mission.sh combined capture
bash scripts/redcho_single_mission.sh combined preview
```

`preview` uses the captured geometry and executes the same numerical REDCHO/HOCBF-QP screening as `offline`. It sends no flight commands. On PASS, stop standalone mocap in terminal A with Ctrl+C. The backend below starts its own mocap path. Recapture after changing configuration or moving aircraft/obstacles.

## 8. Hardware backend, agents and CSV telemetry with all plots off

Stop the standalone mocap first. Start only one backend terminal. If startup
reports an existing endpoint, do not launch another backend: while all aircraft
are grounded and disarmed, close the old backend/mocap terminal with Ctrl+C.
An old backend left running before the Ctrl+C supervisor was added may require
targeted cleanup after checking its PID/executable. Never use a global node kill
during flight. Startup reports are saved under `results/single_backend_startup/`.

Terminal A:

```bash
bash scripts/redcho_single_mission.sh combined backend
```

Terminal B:

```bash
bash scripts/redcho_single_mission.sh combined agents
```

Terminal C:

```bash
bash scripts/redcho_single_mission.sh combined observe \
  recorders:=true capture:=false \
  flight:=false battery:=false timing:=false reference:=false \
  position_redcho:=false control:=false hocbf:=false
```

This keeps the **flight, reference, REDCHO and control CSV recorders**. `flight:=false` disables the flight plot, not the flight recorder. `capture:=false` disables the additional live-plot event recorder, not aircraft pose capture or CSV telemetry. All seven window switches must be false to disable every GUI. To also retain raw live-plot replay events without displaying windows, change only `capture:=false` to `capture:=true`.

## 9. Ground-only readiness and telemetry checks

Terminal D:

```bash
bash scripts/redcho_single_mission.sh combined doctor
bash scripts/redcho_single_mission.sh combined check
```

`doctor` checks local software/interfaces. `check` checks the live ground state, measured streams, firmware parameter cache, numerical captured plan and synthetic agent exchanges. It does not arm, take off, stream position commands or land. Numerical and algorithm telemetry will be generated during its screening/shadow cycles; a ready agent alone need not continuously publish algorithm results.

Optional terminal E, using the same hardware ROS environment:

```bash
cd ~/safe_redcho_hardware
source scripts/environment.bash
source install_single_position/local_setup.bash
ros2 node list
ros2 topic hz /poses
```

Stop each `hz` command with Ctrl+C before the next:

```bash
ros2 topic hz /cf2/pose
ros2 topic hz /cf3/pose
ros2 topic hz /cf4/pose
ros2 topic echo /cf3/status --once
```

The configured onboard pose rate is 20 Hz. A mean rate alone does not rule out the 0.35 s gaps that caused the earlier cf3 abort; the controller's freshness guards and check result remain relevant. Algorithm topic samples can be inspected while check/mission cycles are actually running:

```bash
ros2 topic echo /safe_redcho_single_mission/cf3/algorithm_telemetry --once
```

Stop extra diagnostic subscriptions when finished. If a required check or preview fails, inspect its report before running a flight mode.

## 10. Physical acquisition commissioning

Keep backend, agents/watchdog and recorders running. Terminal D:

```bash
bash scripts/redcho_single_mission.sh combined acquire
```

This sends physical takeoff/formation/landing commands. Verify touchdown/disarm before moving aircraft, restarting processes or beginning another run. Recapture and repeat preview/check if the landed starts or obstacles changed.

## 11. Full physical mission

After successful acquisition commissioning and current preview/readiness checks, terminal D:

```bash
bash scripts/redcho_single_mission.sh combined fly
```

This executes takeoff, warmup, common height, yaw alignment, formation acquisition, forward transport, final hold and landing. No live plot windows are needed; terminal C records the CSV streams throughout.

## 12. Save and inspect results

After touchdown/disarm is verified, stop observers with Ctrl+C to flush their writers. Keep backend and agents/watchdog alive through recovery; stop them only after verified landing/disarm.

```bash
ls -td results/single_mission_combined_check_*/ | head -n 1
ls -td results/single_mission_combined_acquire_*/ | head -n 1
ls -td results/single_mission_combined_fly_*/ | head -n 1
telemetry_run=$(ls -td results/single_mission_telemetry/*/ | head -n 1)
rg --files "$telemetry_run" \
  -g '*.csv' -g 'recorder_status.json' -g 'writer_summary.json'
cat "$telemetry_run"/*/writer_summary.json
```

The mission directory contains `mission_status.json`, effective configuration, numerical preview evidence, `position_commands.csv` and `flight_trace.npz` when produced. Telemetry outputs include configuration snapshots, column dictionaries, recorder status and writer summaries. Inspect failure/recovery evidence and missing/dropped records as well as the final status. Published host commands are not verified onboard RF receipt.
