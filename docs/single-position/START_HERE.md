# Single position REDCHO hardware: installation and commands

This edition adds the separate ROS package `safe_redcho_single_mission` beside
the existing packages. Its source is the restored 0.3.0 controller plus the
requested scoped Ctrl+C cleanup and duplicate-backend startup guard. It has one
order-2 position REDCHO bank per aircraft, battery-aware reference timing,
formation acquisition and transport, visibility guidance and hard HOCBF-QPs.
Position-bank velocity/acceleration outputs replace the independent velocity
bank. The aircraft interface remains position plus yaw.

## 1. Clone and test offline

Ubuntu 24.04, Python 3.12:

```bash
git clone --branch single-position-v0.3.0 --depth 1 \
  https://github.com/khubaib007/Safe-Redcho-Crazyflies.git safe-redcho-single-position
cd safe-redcho-single-position
bash scripts/setup.sh offline
bash scripts/verify.sh
bash scripts/test.sh
bash scripts/redcho_single_mission.sh combined offline
```

These commands use example fixtures and numerical dynamics. They do not start
ROS nodes, open radios or send aircraft commands. `scripts/test.sh` also runs
the package tests, including ROS adapter doubles and scoped process-cleanup
tests. Legacy distribution-installer tests are skipped because this Git checkout
uses the repository installation workflow.

Other numerical fixtures:

```bash
bash scripts/redcho_single_mission.sh default offline
bash scripts/redcho_single_mission.sh obstacle offline
bash scripts/redcho_single_mission.sh interagent offline
```

Reports are saved under `results/`. A numerical PASS screens that fixture and
tracking model; it does not prove physical tracking, RF delivery or flight safety.

## 2. Install ROS and pinned backends

Install ROS 2 Jazzy using the [ROS installation guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html),
then from a clean terminal in the checkout:

```bash
bash scripts/setup.sh ros
/usr/bin/python3 scripts/fetch_dependencies.py
bash scripts/resolve_dependencies.sh
bash scripts/build.sh
bash scripts/verify.sh --ros
bash scripts/redcho_single_mission.sh combined init
```

The dependency lock and existing backend patches are retained from the hardware
repository. The build includes the separate single-bank package. For USB rules
and network setup, use [installation.md](../installation.md).

`init` creates editable, ignored files under `config/local/` and preserves any
existing edits. Set the actual radio URIs, network addresses, arena/floor
calibration and mission parameters there. Root `config/*.yaml` files are
examples. No captured aircraft starts are shipped or fabricated by initialization.
Do not copy generated survey pointers from another checkout.

The portable repository adapter routes live commands through `config/local/`;
the package's original numerical/control code is unchanged. The root checkout
may be placed anywhere and does not need the original workstation path.

## 3. Capture and preview the current floor geometry

Use the same scenario in every terminal. Keep aircraft grounded/disarmed and
stationary. Terminal A:

```bash
bash scripts/redcho_single_mission.sh combined mocap
```

Terminal B:

```bash
bash scripts/redcho_single_mission.sh combined capture
bash scripts/redcho_single_mission.sh combined preview
```

Require preview PASS. Stop standalone mocap in A with Ctrl+C before starting
the backend, which starts its own mocap node. Recapture after moving aircraft
or obstacles or changing source profiles. Every scenario requires a real survey.

## 4. Start one hardware stack

Terminal A:

```bash
bash scripts/redcho_single_mission.sh combined backend
```

Terminal B:

```bash
bash scripts/redcho_single_mission.sh combined agents
```

Terminal C, CSV telemetry with every live plot off:

```bash
bash scripts/redcho_single_mission.sh combined observe \
  recorders:=true capture:=false \
  flight:=false battery:=false timing:=false reference:=false \
  position_redcho:=false control:=false hocbf:=false
```

These flags keep all four CSV recorders; `flight:=false` and `control:=false`
disable plot windows, not recording. `capture:=false` disables the optional
live-plot event capture. Algorithm telemetry is available during actual cycles.
The backend is the only radio/log-setup owner.

## 5. Ground checks before physical flight

Terminal D:

```bash
bash scripts/redcho_single_mission.sh combined doctor
bash scripts/redcho_single_mission.sh combined check
```

`doctor` checks local software/interfaces. `check` verifies the current graph,
measurements, agents/watchdog, parameter cache, captured numerical route and
shadow cycles. It does not arm, take off or stream physical position commands.
Require a current PASS before commissioning.

## 6. Deliberate physical commands

Keep backend, agents/watchdog and recorders running. Terminal D:

```bash
# Acquisition/formation commissioning, followed by landing:
bash scripts/redcho_single_mission.sh combined acquire

# Full mission, only after successful commissioning and fresh checks:
bash scripts/redcho_single_mission.sh combined fly
```

These commands move real aircraft. The stages include takeoff, warmup, common
height, yaw alignment, formation, settling, forward transport, stop and landing.
Confirm touchdown/disarm before moving aircraft or restarting the stack.

Ctrl+C immediately interrupts only the script's owned process group. Non-flight
commands use bounded escalation for hung children. `fly`/`acquire` retain the
landing/recovery routine until it exits; their recovery is not force-killed.
Keep the backend and watchdog alive through recovery. Direct `ros2 launch`
commands bypass the shell wrapper's process-group supervisor.

## 7. Tests and troubleshooting

Focused offline CBF-QP tests:

```bash
source scripts/environment.bash
/usr/bin/python3 -s -m pytest -q \
  src/safe_redcho_single_mission/test/test_predictive_barriers.py \
  src/safe_redcho_single_mission/test/test_qp_priority.py \
  src/safe_redcho_single_mission/test/test_velocity_viability.py \
  src/safe_redcho_single_mission/test/test_responsibility.py \
  src/safe_redcho_single_mission/test/test_passing_encounters.py \
  src/safe_redcho_single_mission/test/test_route_guidance.py
```

Telemetry, adapters, ownership and single-bank tests:

```bash
source scripts/environment.bash
/usr/bin/python3 -s -m pytest -q \
  src/safe_redcho_single_mission/test/test_ros_adapters.py \
  src/safe_redcho_single_mission/test/test_command_routes.py \
  src/safe_redcho_single_mission/test/test_node_shutdown.py \
  src/safe_redcho_single_mission/test/test_plot_data.py \
  src/safe_redcho_single_mission/test/test_single_position.py
```

Actual DDS/telemetry test without aircraft: after building, use two fresh
terminals, and run this preparation in both:

```bash
source scripts/environment.bash
source scripts/ros_environment.bash
export ROS_DOMAIN_ID=174
```

Test terminal 1:

```bash
ros2 launch safe_redcho_single_mission observe.launch.py \
  config_file:="$PWD/config/single_mission_combined.yaml" \
  telemetry_config:="$PWD/config/single_mission_telemetry.yaml" \
  plot_config:="$PWD/config/single_mission_liveplots.yaml" \
  recorders:=true capture:=false \
  flight:=false battery:=false timing:=false reference:=false \
  position_redcho:=false control:=false hocbf:=false
```

Test terminal 2:

```bash
/usr/bin/python3 -s src/safe_redcho_single_mission/test/inert_transport_check.py \
  --config config/single_mission_combined.yaml \
  --starts config/single_mission_combined_fixture.yaml \
  --cycles 250 --output results/single_dds_quick
```

Use a fresh output directory for a repeat. Stop test observers with Ctrl+C to
flush writers, and close both test terminals. Do not start the hardware backend
or a physical mission on test domain 174. Missing physical pose/battery streams
are expected in this synthetic test.

| Symptom | Action |
|---|---|
| Backend startup reports existing receiver | While grounded/disarmed, close the obsolete backend terminal; keep exactly one stack. Inspect `results/single_backend_startup/`. Do not bypass the ownership guard. |
| Existing `/poses` publisher | Stop standalone survey mocap before backend startup; investigate duplicate mocap instances. |
| `backend subscribers=2` | Two radio backends overlap. Close the obsolete one only after confirming physical state. No safety/QP relaxation is needed. |
| Watchdog/agent not ready | Start `combined agents` with the same captured scenario; verify exactly three distinct matching agents and one watchdog. |
| Survey/profile mismatch | Stop control while grounded, recapture and preview current geometry, then restart the stack. |
| Avoidance stalled or QP infeasible | Inspect failed preview geometry, required separation and obstacle inflation. Reposition grounded starts/goals or retune deliberately, then repeat tests and capture/preview. |
| Tracking-tube or stale-measurement abort | Inspect command, measured pose and timing logs. Address tracking/measurement performance; this release keeps the original bounds and does not certify a clean physical flight. |
| No live plots | All plot flags above are deliberately false. CSV recorders remain active. |

Mission reports, effective profiles, command traces and shutdown reports are
under `results/`; CSV outputs include configuration snapshots and writer/drop
statistics. See [validation and provenance](VALIDATION.md) and
[the full mathematical formulation](../../src/safe_redcho_single_mission/MATHEMATICAL_FORMULATION.md).
