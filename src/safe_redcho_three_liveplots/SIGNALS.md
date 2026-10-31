# Signal sources and meanings

| Quantity | Source | Meaning |
|---|---|---|
| Measured position/RPY | `/poses` NamedPoseArray | OptiTrack world pose; unique body names, valid quaternions, source age checked |
| Measured speed | Window fit of `/poses` position vs source timestamp | Derived speed, not a directly received onboard velocity; uses profile's velocity window |
| Onboard Z | `/cfN/pose` PoseStamped | Received onboard estimate; has a separate source-age curve |
| Published position | Existing mission `position_commands.csv` | Host Position publication only; XY/Z values actually written after publication |
| Tracking error | Historical mocap at publication timestamp minus CSV position | Interpolation inside bounded brackets or nearest pose within 40 ms; unavailable otherwise |
| Pair distances | Complete, same-frame mocap batch | XY Euclidean distances; no latest-value mixing across bodies |
| Battery voltage | `/cfN/status.battery_voltage` | Measured Status voltage; governor input voltage is plotted separately |
| Governor/SVF | `/safe_redcho/cfN/algorithm_telemetry`, `data.governor`, `data.svf` | Exact supplied instrumentation: request, governed velocity, filtered velocity/acceleration/jerk |
| Velocity bank | `data.velocity_redcho.input/output` | Velocity, acceleration, jerk in selected axis |
| Position bank | `data.position_redcho.input/output` | Displacement/height OFFSET, velocity, acceleration; not absolute slot position |
| Nominal controller | `data.nominal` and `data.qp.nominal_acceleration_mps2` | Virtual input position/velocity; target, desired velocity, feedforward and nominal acceleration |
| QP output | `data.qp.filtered_acceleration_mps2` | Filtered virtual reference acceleration; Position messages do not transmit it |
| QP intervention | `data.qp.intervention_norm_mps2` | Norm of filtered minus nominal acceleration; smoothing/bounds can cause it |
| HOCBF residual | `data.qp.rows.*.residual_b_minus_Au` | Minimum normalized row residual in m/s²; nonnegative is feasible at that row |
| Pair/obstacle h, psi1 | `data.qp.rows.pair_* / obstacle_*` | Squared-distance barrier (m²), psi1 (m²/s), virtual states |
| Wall h, psi1 | `data.qp.rows.upper_* / lower_*` | Linear-distance barrier (m), psi1 (m/s), virtual states |
| Solver time/flags | `data.qp.solver_wall_s`, flags | Actual QP solve timing and binding/invoked/success fields, not overall cycle runtime |
| Optional firmware fields | Already-enabled blocks mapped by telemetry_three.yaml → onboard_config_file | Read-only samples of configured firmware fields; no log creation, conversion or packet-ack claim |

The decoder matches schema_version=1, exact aircraft name and the profile hash
algorithm used by the attached telemetry package. Profile fields starting with
`_` are excluded from the hash. Restart capture if flight YAML changes.

Convergence requires **all three agents from the same session, sequence, profile,
stage, flight-active flag and streaming flag**. For each bank quantity:

- Mean input: arithmetic mean of the three XYZ inputs.
- Agreement: maximum pairwise Euclidean norm between the three XYZ outputs.
- Reconstruction error: maximum norm of output minus mean input.

These are separate errors, with derivative-specific units. No mixed-cycle or
partial batch is labelled converged. Incomplete batches are bounded/evicted and
reported. There is no hardcoded plot-based convergence PASS threshold.

Clock use:

- Plot X is source ROS time relative to capture start where available; otherwise
  explicitly receipt ROS time (heartbeat/frame intervals and ages).
- Pose/command matching uses ROS source/publication timestamps, not wall-time polling.
- Native command gap uses original publication monotonic timestamps from CSV.
- Age status checks both source age (where available) and receipt monotonic age.
- Algorithm session+sequence is the association key. Position messages/CSV do not
  supply that key; no exact algorithm-cycle association is fabricated.
- ROS backward jumps create a new plot epoch. CSV command observation assumes
  capture runs on the same host/filesystem as the mission. Cross-host clock
  synchronization is not implemented by this plotting package.
