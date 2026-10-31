# Separate live plots for three Crazyflies

ROS 2 Jazzy / Ubuntu 24.04 package: **safe_redcho_three_liveplots**.
Aircraft: **cf2, cf3, cf4**, Position command interface.

Nine independent processes: one capture bridge and eight GUI plot nodes.
You can open one plot, any selection, or all eight with one launch file.
The attached `safe_redcho_three_telemetry` package and the flight software are
not modified by installation. This package contains no arming, flight, landing,
emergency, firmware-parameter or radio-logging setup clients.

## Installation

Download `safe_redcho_three_liveplots.zip` into Downloads. With existing
aircraft experiments stopped, install the GUI dependencies and build:

```bash
cd "$HOME/safe_redcho_hardware"
set +u
source scripts/environment.bash

sudo apt update
sudo apt install python3-pyqtgraph python3-pyqt5 python3-numpy python3-yaml

unzip -o "$HOME/Downloads/safe_redcho_three_liveplots.zip" \
  -d "$HOME/safe_redcho_hardware/src"

colcon build --symlink-install --packages-select safe_redcho_three_liveplots
source install/setup.bash

cp -n src/safe_redcho_three_liveplots/config/liveplots.yaml \
  config/liveplots_three.yaml

ros2 pkg executables safe_redcho_three_liveplots
```

Stop if a command fails. The package uses system Qt packages, not a replacement
ROS Python environment. If your virtual environment hides the apt packages,
verify `/usr/bin/python3 -c 'import pyqtgraph; from PyQt5 import QtWidgets'`
and build with `/usr/bin/colcon` (the ROS/apt Python interpreter). Do not install
a different NumPy into the flight environment merely for these plots.

Run GUI nodes on the Ubuntu desktop with a working DISPLAY/Wayland session.
Only `live_capture` can run without a graphical display.

## Use with the existing experiment

Keep the usual backend and agents in their separate terminals. Algorithm
instrumentation must be enabled **when starting the agents**:

```bash
cd "$HOME/safe_redcho_hardware"
set +u
source scripts/environment.bash
SAFE_REDCHO_TELEMETRY=1 bash +u scripts/redcho_three.sh agents
```

Then, in a separate plot terminal:

```bash
cd "$HOME/safe_redcho_hardware"
set +u
source scripts/environment.bash
source install/setup.bash

ros2 launch safe_redcho_three_liveplots all.launch.py \
  config_file:="$HOME/safe_redcho_hardware/config/liveplots_three.yaml"
```

This starts capture plus all eight windows. It sends no flight commands.
Use the existing `check` and `hover`/`fly` terminals to control your experiment.
Closing a plot window stops that viewer only. Ctrl+C in the launch terminal
stops the plotting processes; it is not a landing command.

For a lighter first run, launch just flight, battery and timing:

```bash
ros2 launch safe_redcho_three_liveplots all.launch.py \
  config_file:="$HOME/safe_redcho_hardware/config/liveplots_three.yaml" \
  reference:=false velocity_redcho:=false position_redcho:=false \
  control:=false hocbf:=false
```

## Run each node separately

Start capture **once**, before starting a mission:

```bash
cd "$HOME/safe_redcho_hardware"
set +u
source scripts/environment.bash
source install/setup.bash

ros2 run safe_redcho_three_liveplots live_capture \
  --config "$HOME/safe_redcho_hardware/config/liveplots_three.yaml"
```

In each additional terminal, source the same environment and run one viewer:

```bash
ros2 run safe_redcho_three_liveplots flight_plot --config "$HOME/safe_redcho_hardware/config/liveplots_three.yaml"
```

Replace `flight_plot` with the desired executable:

| Executable | Plots |
|---|---|
| `flight_plot` | Height, XY paths, time-aligned tracking error, speed, pair spacing, roll/pitch/yaw, onboard vs mocap height |
| `battery_plot` | Battery voltage, governor input voltage and governor speed scale |
| `timing_plot` | Source ages, original host command intervals, observed frame intervals, heartbeat age and emitter drops |
| `reference_plot` | Governor input/output; SVF velocity, acceleration and jerk |
| `velocity_redcho_plot` | Bank input/output and exact-cycle agreement/mean-reconstruction error for velocity, acceleration and jerk |
| `position_redcho_plot` | Bank input/output and exact-cycle convergence for displacement/height offset, velocity and acceleration |
| `control_plot` | Nominal-controller inputs/targets, feedforward, nominal vs filtered acceleration and QP correction |
| `hocbf_plot` | Normalized row residual, separate pair/obstacle and wall barriers, psi1, violation, binding/solver flags and solver time |

If capture is already running, use `capture:=false` when launching more viewers:

```bash
ros2 launch safe_redcho_three_liveplots all.launch.py \
  config_file:="$HOME/safe_redcho_hardware/config/liveplots_three.yaml" capture:=false
```

Do not start a second capture node. A viewer refuses data if it discovers more
than one publisher on its plot topic.

## Important: actual outgoing setpoints without a command subscriber

The mission's preflight permits only its backend and the known existing control
recorder as subscribers on `/cfN/cmd_position`. Adding a plot subscriber would
fail that preflight. Therefore this package does **not** subscribe there and
does not spoof the permitted recorder's node name.

Instead, capture tails the existing line-buffered mission file:

`~/safe_redcho_hardware/results/three_*/position_commands.csv`

It automatically selects the most recently active file modified after capture
starts. Discovery/polling is every 100 ms; drawings/transport refresh at 5 Hz.
The command curves have file/transport/display latency, not guaranteed subcycle
latency. Command timestamps and intervals are taken from the original CSV rows;
the file reader's polling interval is never reported as the command interval.

For a custom results directory or a specific running mission, set `command_file`
in the plotting YAML, or start capture with:

```bash
ros2 run safe_redcho_three_liveplots live_capture \
  --config "$HOME/safe_redcho_hardware/config/liveplots_three.yaml" \
  --command-file "$HOME/safe_redcho_hardware/results/YOUR_RUN/position_commands.csv"
```

The file can appear after capture starts. Rows older than `command_max_age_s`
(default 2 s) are discarded. This is a **display-reader freshness limit**, not
the flight publication-gap guard. If the file is missing, command and tracking
curves remain MISSING; virtual references are not substituted as publications.
The old telemetry recorder's wide/long CSV is not used as a silent fallback.
Existing CSV recorders may continue running, but are not required for these plots.

## Display controls

- cf2 cyan, cf3 amber, cf4 pink across every window.
- Input/request/reference comparison curves are dashed; corresponding outputs
  are solid. Legends identify each quantity.
- Select X/Y/Z in the algorithm windows. Position-bank displacement is relative
  to the formation slot; Z is the height offset, not absolute flight height.
- A stage banner and vertical stage markers show the most recently observed
  algorithm stage or CSV stage. They are not a new authoritative mission-status
  channel. Takeoff/landing stages may show NO FRESH ALGORITHM FRAME.
- SHADOW explicitly means computation without flight command application.
- Pause stops drawing; acquisition and bounded history continue.
- Clear empties the view history. Save PNG stores a screenshot in
  `results/liveplots/snapshots`.
- Missing samples become gaps, never fabricated zeros. Stale histories remain
  visible with a STALE banner. A genuine zero velocity bank in hover remains zero.

Optional sampled onboard fields appear in Flight/Control only when the existing
telemetry configuration maps already-enabled log blocks. Choose a field from
the Onboard field menu. Values are raw/configured firmware fields, without
unrequested unit conversions. This package never enables a log block; do not
add RF traffic simply to fill every panel during a troubled flight.

## Recording and timing limits

`record_events: true` records the normalized plotted quantities at incoming
sample rates into separate JSONL files under `results/liveplots/<UTC run>/`.
It records the plotted subset, not every internal state from the telemetry
package. Use the existing telemetry recorders for the complete raw dataset.

Each event retains source/receipt ROS nanoseconds, receipt monotonic nanoseconds,
and available session/sequence/stage/flight flags. Profile hash/configuration is
saved. Command events identify the source file and publication monotonic time.
No host command receipt is represented as an onboard packet acknowledgement.

Only the GUI transport/drawing is refreshed at 5 Hz. Samples within each batch
are retained individually; this is not an average that erases short spikes.
Per-signal history is bounded by 90 s **and** 6000 points; a 120 Hz channel thus
retains about 50 s at the defaults. Reduce refresh/window limits if the laptop
is loaded. Keep checking the experiment's real publication and watchdog budgets.

Best-effort DDS, bounded queues and a stalled process can lose samples. The
banner reports sequence gaps, incomplete convergence batches, parser rejections,
writer/capture drops and viewer batch loss. `summary.json` and
`capture_summary.json` record shutdown diagnostics. Source-time pose matching
has a maximum 40 ms nearest-neighbor skew (or bounded bracket interpolation).
Tracking error shown here is a time-aligned host diagnostic, not an exact replay
of the mission's internal guard. Different clocks are not treated as interchangeable.

## What these plots cannot prove

- Published Position is not confirmed onboard receipt or the final PID/motor output.
- Frame inter-arrival time is not full controller CPU execution time. Full mission
  cycle execution time is not published by the supplied instrumentation and is
  deliberately not invented here. QP solver execution time is available.
- A QP correction or solver invocation alone does not prove obstacle avoidance.
  Check binding rows and residuals. HOCBF curves concern virtual references.
- Observer agreement is different from reconstruction accuracy: both are plotted.
- Computed algorithm/QP telemetry does not prove that a complete batch was
  accepted by the mission or applied onboard. Producer errors are shown in the banner.
- Algorithm panels normally have no new data during takeoff/handover. Both banks
  can be zero at hover. No synthetic data is mixed into the live ROS topics.
- These viewers do not repair cf4's descent, telemetry gaps, RF loss or flight bugs.

Read `SIGNALS.md` for the exact source/quantity mappings.

## Offline validation

```bash
cd "$HOME/safe_redcho_hardware"
python3 -m unittest discover -s src/safe_redcho_three_liveplots/test -v
```

The package was tested with deterministic data/CSV tests, the supplied message
schema, and offscreen Qt rendering of all eight windows. No ROS runtime, radio
or hardware flight was available here. The screenshots generated during QA
used labelled test fixtures, not experimental flight data.
