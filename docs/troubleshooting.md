# Troubleshooting

| Symptom | Inspect | Resolution |
|---|---|---|
| SciPy error mentioning numpy.long | Numerical versions from verify.sh | Use wrappers/system Python with user-site isolation; exit unrelated venv/Conda. |
| Dependency commit/patch mismatch | fetch_dependencies.py --check | Preserve local edits; obtain the pinned checkout and exact patches. Do not reset a working dependency automatically. |
| Missing ROS interface/backend executable | verify.sh --ros and first build error | Fetch dependencies, install prerequisites and build this edition. Do not reuse another checkout's install tree. |
| Wrong overlay/package prefix | Prefix printed by verification | Use a fresh terminal and this checkout's wrappers. |
| USB permission denied | lsusb and udev/group access | Apply the supplied USB rules once and replug/login as appropriate. Close cfclient and other radio owners. |
| No /poses | Motive streaming, wired route, multicast and firewall | Verify receiver/backend, exact IP/ports and wired source. Do not run duplicate receivers. |
| Wrong coordinates or yaw | Hand-motion/body-axis checks with props removed | Correct Motive origin, scale, pivot and axes; changing frame_id is insufficient. |
| Missing/duplicate rigid body | Exact Motive body names | Match active aircraft names and unique enabled bodies. |
| Stale onboard pose but fresh mocap | Source/receipt ages, backend/RF logs | Diagnose radio/logging/firmware delivery. Do not merely raise age limits. |
| Invalid huge estimator position | Raw onboard logs and mismatch report | Stop flight commissioning and verify estimator/backend decoding and firmware compatibility. |
| Wrong source-profile hash / missing algorithm plots | Effective profile paths and bad-packet counters | Use the same selected edition/scenario. Mission observe resolves the same immutable survey as control. |
| Source YAML changed after survey | Survey resolver error | Stop control processes on the ground and recapture; do not edit immutable outputs. |
| No survey for scenario | Generated profile missing | Perform that scenario's ground survey before backend/agents/check/observe. |
| QP or route preview infeasible | Failed row, inflated geometry and starts | Correct geometry/layout/assignment or planning duration; keep hard constraints enabled. |
| Tracking-tube departure | Measured/reference trace and command cadence | Investigate actual tracking and RF/onboard behavior before changing allowances. |
| Low voltage abort | Received status age and actual battery | Replace/recharge and recheck; do not substitute a fabricated voltage. |
| Competing command route | Publishers, active names, old launches | Stop the other stack after landing; run one flight stack at a time. |
| Qt pen enum error | Selected Qt binding | This release fixes legacy PenStyle access. Use supplied offline dependencies and the current viewer. |
| Check-only PASS then flight fails | Mission status, caches, timestamps and physical gates | Ground checks are necessary evidence, not proof of physical command execution. |

For an issue include tag, OS/ROS, firmware and radio/Motive versions, effective
profile, dependency-lock check, first failure and timestamped reports. Do not
upload config/local or unreviewed experiment data with unrelated local files.
