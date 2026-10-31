# safe_redcho_three_mission

A separate ROS 2 Jazzy / Crazyswarm2 C++ package for cf2, cf3 and cf4. It adds
individual takeoff, REDCHO height agreement, smooth world-X heading alignment,
charge-aware formation acquisition and the forward mission. Both REDCHO banks
compute every streamed control cycle, including zero-translation stages.

This release is software-validated. ROS adapters were tested with test doubles;
ROS discovery, actual radio delivery, firmware behavior and physical flight have
not been tested by the author in this environment. A PASS in the synthetic
preview is a necessary screening result, not hardware certification.

## Installation and preservation of the previous implementation

The installer does not modify the old package, launchers, flight YAML or radio
URIs. It first copies the old source packages, scripts and config to a dated
`results/checkpoints/before_three_mission_*` directory with SHA256 hashes and
read-only backup files. The live original files remain available. Build/install
products, complete third-party repositories and experiment results are not part
of this checkpoint. Keep your own firmware and upstream Git revisions as well.

Stop existing flight/observer launches after verified landing before changing
packages. Download the bundle ZIP to Downloads, then run:

```bash
cd "$HOME/safe_redcho_hardware"
set +u
source scripts/environment.bash
export PYTHONNOUSERSITE=1

unzip -o "$HOME/Downloads/safe_redcho_three_mission_bundle.zip" -d "$HOME/Downloads"

/usr/bin/python3 "$HOME/Downloads/safe_redcho_three_mission_bundle/install.py" \
  --workspace "$HOME/safe_redcho_hardware"

sudo apt update
sudo apt install python3-numpy python3-scipy python3-yaml python3-pyqt5 python3-pyqtgraph

/usr/bin/colcon build --symlink-install --packages-select safe_redcho_three_mission
source install/setup.bash

bash +u scripts/redcho_three_mission.sh doctor
bash +u scripts/redcho_three_mission.sh offline
```

The installer refuses to overwrite an existing new-package target. It inherits
the measured arena from `config/redcho_three.yaml` when present, validates that
the new mission fits, and leaves the old YAML unchanged. It reads the active URIs
from your current `config/fleet_redcho.yaml`; check cf2/cf3/cf4 there. The previous
six-aircraft/two-radio validator is not used by the new backend.

Verify the old working files against the latest checkpoint:

```bash
cd "$HOME/safe_redcho_hardware"
/usr/bin/python3 "$HOME/Downloads/safe_redcho_three_mission_bundle/verify_legacy.py" \
  --workspace "$HOME/safe_redcho_hardware"
```

No old build products need to be deleted. The old `scripts/redcho_three.sh`
continues to address the old package; the new script is
`scripts/redcho_three_mission.sh`. Run one flight stack at a time.

## Configuration

Edit only the new experiment configuration:

`~/safe_redcho_hardware/config/three_mission.yaml`

Other newly created files:

- `config/three_mission_telemetry.yaml`: CSV recorders; recording on, periodic terminal values off.
- `config/three_mission_liveplots.yaml`: plot histories and normalized JSONL recording.
- `config/three_mission_onboard.yaml`: optional received-log mappings, disabled by default.
- `config/three_mission_starts.yaml`: written by capture, with world positions and arbitrary yaw.
- `config/generated_three_mission/`: new generated backend configuration only.

Initial center coordinates must be inside X [-0.8, 0] m and Y [-0.8, 0.8] m.
They must also satisfy the full reference separation and airborne volume tests.
There is no fixed floor-slot or world-X initial-yaw requirement. Aircraft still
must be level and stationary, with valid mocap/onboard alignment.

Default values:

| Setting | Value / role |
|---|---|
| Individual heights above calibrated floor | cf2 0.40 m; cf3 0.50 m; cf4 0.60 m |
| Final common height | Mean of the three measured settled hover-height proposals |
| Formation center XY | [0.15, 0.0] m |
| Formation row offsets | Y +0.95, 0, -0.95 m; centered X offsets 0 |
| Assignment | `sorted_y`: upper initial aircraft gets upper slot, etc. |
| Acquisition nominal duration | 24 s before voltage scaling and settling |
| Forward displacement / nominal duration | 0.60 m / 15 s |
| Position period | Target 0.02 s; no catch-up publication bursts |
| Reference speed | Componentwise bound 0.18 m/s |
| Measured speed guard | 0.40 m/s in 3D norm |
| Physical / measured abort / virtual XY separation | 0.50 / 0.65 / 0.91 m |
| Tracking tube | 0.14 m in 3D norm |
| Yaw rate / final yaw error | 0.60 rad/s / 0.20 rad |
| Endpoint error / speed | 0.10 m / 0.10 m/s with dwell |
| Mocap / onboard maximum age | 0.08 / 0.30 s |
| Ground / land voltage | 3.70 / 3.30 V |
| Shape clock minimum voltage scale | 0.65 |

The numerical example uses floor XY (-0.05,0.76), (-0.78,0), (-0.05,-0.76) m
with yaw 2.5, -1.8 and 0.9 rad. These are an example, not mandatory slots.
Random points chosen close together are rejected even if they are in the box.
The separation certificate is horizontal; different heights do not waive it.

Set `acquisition.assignment: fixed_names` only when you need the original name
to retain its configured slot. This can introduce crossing/blocked goals, which
the numerical preview may reject. This release has no general obstacle-routing
planner and requires an obstacle-free measured arena. It does not guarantee
that every geometrically valid random arrangement can acquire every assignment.

The static profile checks the nominal formation sweep. The captured-start plan
also checks the negotiated-height sweep. Default final X is 0.75 m, within the
inflated volume. A 1 m move from X=0.15 does not fit the previous margins in this
3 m room; increase the allowed room only from actual measurements.

## Complete command sequence

The new shell script sources the existing environment and installed overlay
with nounset disabled around setup. Begin commands in the main workspace.

1. Terminal A: standalone mocap for capturing starts.

```bash
cd "$HOME/safe_redcho_hardware"
bash +u scripts/redcho_three_mission.sh mocap
```

2. Terminal B: capture the three grounded aircraft, including arbitrary yaw.

```bash
cd "$HOME/safe_redcho_hardware"
bash +u scripts/redcho_three_mission.sh capture
bash +u scripts/redcho_three_mission.sh preview
```

`preview` uses the newly captured positions and headings and sends no radio or
flight commands. `offline` instead uses the supplied synthetic example.
After capture/preview PASS, stop the standalone mocap launch with Ctrl+C.

3. Terminal A: start the NEW backend; leave it running.

```bash
cd "$HOME/safe_redcho_hardware"
bash +u scripts/redcho_three_mission.sh backend
```

4. Terminal B: start three agents and the independent watchdog; leave running.

```bash
cd "$HOME/safe_redcho_hardware"
bash +u scripts/redcho_three_mission.sh agents
```

Algorithm telemetry publication is built into this new package; no environment
flag is needed. Each agent owns its two banks and its local hard QP.

5. Terminal C: start four quiet CSV recorders, live capture and all eight plots.

```bash
cd "$HOME/safe_redcho_hardware"
bash +u scripts/redcho_three_mission.sh observe
```

For a lighter GUI with all CSV recording still enabled:

```bash
cd "$HOME/safe_redcho_hardware"
bash +u scripts/redcho_three_mission.sh observe \
  reference:=false velocity_redcho:=false position_redcho:=false \
  control:=false hocbf:=false
```

A false plot argument disables that window, not the corresponding recorder.
Recorders use a shared run directory; their periodic terminal values default to
false. Startup messages, missing/stale warnings and flight/watchdog messages
remain visible. Do not start the old observers at the same time.

6. Terminal D: ground-only check with real ROS agents, firmware cache checks and
numerical acquisition screening. No arm/takeoff/position/land commands are sent.

```bash
cd "$HOME/safe_redcho_hardware"
bash +u scripts/redcho_three_mission.sh check
```

7. Hardware acquisition-only commissioning, followed by automatic landing:

```bash
cd "$HOME/safe_redcho_hardware"
bash +u scripts/redcho_three_mission.sh acquire
```

8. Full mission, including the forward translation and landing:

```bash
cd "$HOME/safe_redcho_hardware"
bash +u scripts/redcho_three_mission.sh fly
```

Both flight modes repeat ground checks and preview before arming. `acquire`
and `fly` are physical-flight commands; `doctor`, `offline`, `preview`, `capture`
and `check` are not. There is no unattended launch of the flight coordinator.

After verified landing/disarm, stop observe with Ctrl+C to drain/close files,
then stop agents and backend. Keep the backend and watchdog alive during recovery.

For a deliberately requested motor-cut emergency, a separately sourced terminal
can address the active aircraft individually:

```bash
cd "$HOME/safe_redcho_hardware"
set +u
source scripts/environment.bash
source install/setup.bash
for n in cf2 cf3 cf4; do
  ros2 service call "/$n/emergency" std_srvs/srv/Empty '{}' &
done
wait
```

Emergency cuts motors; it is not a controlled landing.

## What the algorithms actually do

After high-level individual takeoff, the coordinator streams the planned hover
references with measured current yaw, avoiding a snap to yaw zero. Frozen height
proposals are measured after the individual hovers settle. Planned XY anchors
remain fixed; the reference is not silently reset to a drifting measured pose.

Every streamed cycle performs both REDCHO prepare/exchange/update steps:

1. Each agent receives its actual battery voltage in the common cycle frame.
2. Its local voltage governor advances the acquisition or forward quintic phase.
3. An SVF generates displacement, velocity, acceleration and jerk.
4. Velocity REDCHO reconstructs average group velocity and derivatives.
5. Position REDCHO reconstructs average **absolute center/height proposals**.
6. A common smooth shape clock moves the per-aircraft offsets from their initial
   values to their assigned final values. It uses the weakest voltage scale.
7. Desired velocity is REDCHO group velocity plus the local shape derivative;
   desired acceleration includes both corresponding terms.
8. Nominal feedback uses virtual reference position/velocity, followed by a
   local hard reciprocal HOCBF-QP.
9. The mission accepts only a complete same-cycle three-agent acceleration
   batch, validates joint constraints and the held path, and integrates it.
10. Position + yaw is published. Velocity/acceleration/jerk are **not transmitted
    in a Position message**. Onboard PID/BVCA handle the received setpoint.

During WARMUP, position is held and bank estimates are observed. During HEIGHT
and YAW, horizontal desired references remain at the individual hover anchors.
During FORMATION and SETTLE the common center moves toward the designated center
while offsets acquire the formation. FORWARD then adds the configured X motion
without resetting banks, filters, position, velocity or previous acceleration.
STOP lets filters settle and requires a measured final dwell before landing.

The velocity bank describes shared translation. Individual Y motion can be
nonzero even when bank Y velocity is zero, because formation-shape velocity is
separate. See `formation_shape` and nominal desired-velocity telemetry for that
contribution. Independent vectors pointing toward different goals are not
averaged into a single command.

Height/yaw are not made equal by disabling safety or jumping position packets.
Yaw is a known-reference shortest-angle ramp to world +X, not yaw REDCHO.
Bank agreement is a computational gate; measured height, yaw, position and speed
provide separate physical convergence gates.

The averaged voltage-aware group velocity is not a slowest-aircraft velocity
cap. The shape clock uses the weakest scale, but the controller also bounds the
complete virtual reference. Battery scaling uses measured voltage, not an
estimated state-of-charge model.

## Recording, plots and units

The new package contains its own recorder/viewer modules, with no dependency on
or changes to the old telemetry/live-plot packages. Topics use
`/safe_redcho_mission/...`; viewers use `/safe_redcho_mission/live/<group>`.

Results:

- `results/three_mission_check_*`, `three_mission_acquire_*`, `three_mission_fly_*`:
  effective profile, captured plan, parameter-cache report, ground preview,
  mission status, command CSV and flight trace when available.
- `results/three_mission_telemetry/<UTC run>/`: flight/reference/REDCHO/control
  CSV subdirectories, units/dictionaries and recording-health summaries.
- `results/three_mission_liveplots/<UTC run>/`: normalized plot JSONL and capture summary.
- `results/three_mission_watchdog/watchdog_report.json`: independent recovery report.

The command journal is asynchronous and bounded. It records publication times
before enqueueing. Overflow/writer failure causes the mission to recover rather
than silently claim complete recording. Plot command acquisition tails only
new `three_mission_*/position_commands.csv` files. It does not subscribe to the
restricted command topics. The single known control recorder is allowed there.

Algorithm records preserve session, sequence, profile hash and producer clocks.
Position messages do not carry the algorithm sequence; no exact association or
onboard receipt is fabricated. Blank/missing values are not zero. Monitoring
queues are bounded and BEST_EFFORT; inspect their drop/error counters. The
command CSV and telemetry sources are authoritative for publication evidence;
none alone proves actual PID/motor output.

The schema compatibility key `position_redcho.*.displacement_m` now means
**absolute world center/height in metres**, as stated in packet metadata and the
new plot labels. Do not combine it with the old bank's displacement/offset data
without interpreting the semantics. Local shape terms are additional columns.

The NPZ flight trace stores the pre-cycle measured snapshot with the next
published reference; precise source/publication-time alignment should use the
raw telemetry timestamps and command journal. It is not a replay of unlogged
firmware acquisition time. Live tracking curves use bounded historical matching.

Available independent commands after sourcing the workspace:

```bash
ros2 run safe_redcho_three_mission flight_telemetry --config "$HOME/safe_redcho_hardware/config/three_mission_telemetry.yaml"
ros2 run safe_redcho_three_mission reference_telemetry --config "$HOME/safe_redcho_hardware/config/three_mission_telemetry.yaml"
ros2 run safe_redcho_three_mission redcho_telemetry --config "$HOME/safe_redcho_hardware/config/three_mission_telemetry.yaml"
ros2 run safe_redcho_three_mission control_telemetry --config "$HOME/safe_redcho_hardware/config/three_mission_telemetry.yaml"

ros2 run safe_redcho_three_mission live_capture --config "$HOME/safe_redcho_hardware/config/three_mission_liveplots.yaml"
ros2 run safe_redcho_three_mission flight_plot --config "$HOME/safe_redcho_hardware/config/three_mission_liveplots.yaml"
```

Other viewer executables: `battery_plot`, `timing_plot`, `reference_plot`,
`velocity_redcho_plot`, `position_redcho_plot`, `control_plot`, `hocbf_plot`.
Use one live_capture and at most one control recorder. To open extra viewers
while observe runs, pass `capture:=false recorders:=false` and disable windows
already running, or run just the individual executable.

## Safety and validation scope

The physical separation, tracking-tube, stale-data, infeasibility and watchdog
checks remain. Acquisition tolerances/timeouts accommodate the new scenario;
previous fixed-slot and initial-yaw restrictions are replaced with checks that
match arbitrary captured starting poses. They are not blanket-disabled guards.
The airborne minimum-height guard is 0.25 m for the 0.40 m initial hover; actual
floor/ceiling and tracking checks still apply. Review measured room dimensions
and current tracking capability before adjusting any safety allowance.

Ground preview tests deterministic virtual dynamics and a synthetic lagging
plant using captured starts and sampled voltages. Voltage, sensor noise, wind,
RF loss, BVCA intervention and physical transients can differ in flight. The QP
and physical guards continue every cycle. No general multi-agent reachability
or hardware safety guarantee is claimed. A blocked/infeasible preview aborts
before arming. Changing voltage can change the later path/timing.

Firmware preflight reads the ROS server parameter cache sequentially with
bounded retries; it does not change parameters, claim fresh RF readback or verify
BVCA peer-table coverage across your three radio channels. Backend startup
requests PID, Kalman and BVCA settings. Logged onboard targets remain optional;
the stock backend exposes status and pose by default.

The discrete, epsilon-smoothed REDCHO implementation is checked with numerical
agreement tolerances. Continuous-time exact-convergence claims from the REDCHO
paper are not asserted for physical hardware or this sampled implementation.

Tests can be run without ROS (adapters use explicit test doubles):

```bash
cd "$HOME/safe_redcho_hardware"
QT_QPA_PLATFORM=offscreen PYTHONPATH=src/safe_redcho_three_mission \
  /usr/bin/python3 -m unittest discover -s src/safe_redcho_three_mission/test -v
```

Qt tests skip when Qt/PyQtGraph is absent. Included validation results state
whether they ran. See `VALIDATION.txt`, `validation/scenario_results.json` and
`validation/test_results.txt` in this bundle.

Primary interface references:

- REDCHO paper: https://doi.org/10.1016/j.automatica.2022.110320
- Crazyswarm2 Position/yaw API (radians): https://github.com/IMRCLab/crazyswarm2/blob/main/crazyflie_py/crazyflie_py/crazyflie.py
- Firmware controllers: https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/functional-areas/sensor-to-control/controllers/
- BVCA/firmware parameters: https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/api/params/
