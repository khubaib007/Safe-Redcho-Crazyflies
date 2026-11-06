# Single-position REDCHO 0.3.0 route validation — 4 November 2026

## Result and scope

The final source passes 217 tests (2 legacy installer-format tests skipped), 10 standard profile/battery numerical missions and 10 numerical replays of previously failed preview geometries. All numerical missions execute the real single-bank reference generator, local QPs, complete reciprocal/held-path checks, common pacing and measured-stage gates using a synthetic first-order tracker. These are scenario-specific numerical results, not completed hardware flight qualification.

The previous 0.2.0 state is frozen in `../checkpoints/single_before_route_guidance_20261104/` (94 files, all copied hashes verified). Existing dual-bank packages, profiles and launchers were not edited in this task. `source_sha256.json` identifies the 98 current package/profile/launcher files.

## What was corrected

The five archived failed previews completed formation, then deadlocked in STOP during obstacle/peer/right-wall goal recovery. The old circulation guidance lacked a complete clear-goal segment test and a finite detour path. Version 0.3.0 adds bounded visibility graphs around the inflated peer/obstacle disks, Dijkstra shortest paths, waypoint tracking, snapshot replanning and direct-goal recovery using the actual current REDCHO target.

A production trial also exposed loss of an in-progress passing decision when switching guidance modes. Fallback encounter/blend/stall context is now retained across handoffs, and the 18:27:07 geometry plus a focused handoff test protects that behavior. The FORWARD phase can use the configured 80 s route-stage budget when the previous battery-only timeout is too short for deliberate pacing. The 12 s low-motion guard, 180 s overall logical budget and live wall-time bound remain.

No hard safety row, separation, inflation, tracking reserve, acceleration/speed cap or hard safety-slew cap was relaxed. Route guidance changes the nominal XY preference only. Z tracking, one REDCHO bank, SVFs, battery mapping, telemetry, recovery and mission sequence remain.

## Test suite and local software

- `tests.log` / `tests.xml`: **217 passed, 2 skipped**, final full-suite run.

- The suite includes exact recorded full-mission regressions at nominal and unequal voltages, head-on swaps, perpendicular crossings, near-wall interactions, graph geometry/determinism/budget checks, occupied-goal fallback, current-goal release and passing-context continuity.

- A pre-existing allocation-test helper selected the adaptive policy while two comparisons expected the original equal policy. The frozen regression now explicitly selects the policy being tested; no production allocation mathematics changed. Earlier logs are labelled `*_before_final`.

- SciPy emitted internal SLSQP iterate-clipping warnings. Every returned command was independently checked against all final hard rows/bounds and complete-batch checks.

- `build.log`: separate install overlay built successfully as 0.3.0; installed controller modules match source. The interface packages are reused from the existing hardware install.

- `doctor.json`: PASS_LOCAL_SOFTWARE, without creating a ROS node or accessing radios.

## Standard profile matrix

Voltage vectors: nominal `[4.0, 3.8, 3.7]` V; unequal `[3.55, 3.45, 3.35]` V. The fast stress request halves only the combined formation phase from 15 to 7.5 s. Times below are logical streamed-mission times and exclude physical takeoff/landing.

| Scenario | Status | Mission [s] | Formation phase [s] | Min virtual pair [m] | Max synthetic tracking [m] |
|---|---|---:|---:|---:|---:|
| default_nominal | PASS | 49.60 | 15.02 | 0.919872 | 0.015945 |
| default_unequal | PASS | 63.06 | 23.08 | 0.919801 | 0.011756 |
| combined_nominal | PASS | 62.60 | 18.96 | 0.734658 | 0.015091 |
| combined_unequal | PASS | 82.70 | 27.86 | 0.734675 | 0.010827 |
| obstacle_nominal | PASS | 48.02 | 15.02 | 0.788617 | 0.018916 |
| obstacle_unequal | PASS | 61.42 | 23.08 | 0.788617 | 0.014228 |
| interagent_nominal | PASS | 53.16 | 20.62 | 0.734673 | 0.015945 |
| interagent_unequal | PASS | 69.76 | 31.82 | 0.734684 | 0.011756 |
| combined_fast_nominal | PASS | 55.16 | 11.52 | 0.734686 | 0.019030 |
| combined_fast_unequal | PASS | 70.12 | 15.28 | 0.734675 | 0.015192 |

Each `profiles/<case>/` contains its configuration, plan, full report and finite NPZ trace. HOCBF row CSV diagnostics were disabled for these matrix runs; the actual local/joint hard validations still run every cycle.

## Previously failed captured geometries

The archived effective profile and plan are retained exactly except for enabling `avoidance.route_guidance`. No starts, obstacles, safety reserves or mission goals were moved to obtain PASS.

| Recorded preview | Battery | Status | Mission [s] | Min virtual pair [m] |
|---|---|---|---:|---:|
| 171610_105201Z | nominal | PASS | 77.48 | 0.733039 |
| 171610_105201Z | unequal | PASS | 106.30 | 0.733039 |
| 182422_163997Z | nominal | PASS | 69.96 | 0.734691 |
| 182422_163997Z | unequal | PASS | 99.18 | 0.734685 |
| 182707_600308Z | nominal | PASS | 82.62 | 0.734692 |
| 182707_600308Z | unequal | PASS | 94.38 | 0.734680 |
| 194140_919503Z | nominal | PASS | 79.54 | 0.734691 |
| 194140_919503Z | unequal | PASS | 90.44 | 0.734681 |
| 194330_544226Z | nominal | PASS | 78.72 | 0.734691 |
| 194330_544226Z | unequal | PASS | 88.24 | 0.734681 |

Across these 10 replays, the minimum virtual pair distance was **0.733039 m** (required 0.718 m); minimum virtual obstacle-center distance was **0.447301 m** (required 0.439 m); maximum synthetic tracking error was **0.020132 m**; maximum reference slew was **6.299852 m/s³** (hard cap 8).

`archived/` contains 10 report/config/plan/trace sets, aggregate metrics, finite-array audit, a frozen 47-file executable-package snapshot and matching source hashes. One matrix driver was externally terminated after nine completed reports; the remaining case was rerun independently against the identical frozen source. The complete final matrix contains no numerical FAIL.

![Accepted formation and obstacle-detour reference trajectories](route_replay.png)

The plot is a numerical replay of the 18:24:22 geometry. Dots mark initial references; crosses mark goals; dotted formation lines show direct connections; the gray disk is the inflated obstacle region, and the dashed rectangle is the virtual XY reference box. The forward panel includes STOP goal recovery. It is not a physical flight trace. Editable vector export: `route_replay.svg`; reproduction: `plot_routes.py`.

## Installed three-agent ROS/DDS test

**PASS** on isolated ROS domain 174, using three distinct installed agent processes and synthetic tracking. No aircraft position publishers, backend, radios, watchdog recovery services or flight commands are created by the harness.

- Cycles: 4719; logical duration: 94.38 s.
- Maximum complete cycle: 48.51 ms; median: 12.12 ms; configured deadline: 80 ms.
- Telemetry: {'cf2': 4719, 'cf3': 4719, 'cf4': 4719}.
- Route exercise/timing: `{"active_agent_samples": 787, "search_samples": 33, "maximum_search_wall_s": 0.0013469829991663573, "maximum_nodes": 52}`.

Evidence: `transport/full_mission/report.json` and per-agent logs; `transport/profile.yaml`/`starts.yaml` reproduce the 18:27:07 geometry with unequal batteries. All subprocesses are stopped in the harness cleanup. This does not establish a worst-case real-time bound or a guarantee of 20 ms execution on every cycle.

## Reproduction

From the hardware workspace, use the README build instructions, then:

```bash
PYTHONNOUSERSITE=1 PYTHONPATH=src/safe_redcho_single_mission QT_QPA_PLATFORM=offscreen \
  /usr/bin/python3 -s -m pytest src/safe_redcho_single_mission/test -q

PYTHONNOUSERSITE=1 PYTHONPATH=src/safe_redcho_single_mission \
  /usr/bin/python3 -s src/safe_redcho_single_mission/test/validate_scenarios.py \
  --workspace "$PWD" --output results/single_route_manual
```

The recorded regression fixtures are in `test/fixtures/route_regressions.json`; they do not depend on a live survey or aircraft. The manual DDS harness requires an isolated non-production domain and the separate installed overlay; it is not a flight command.

## Limits and required hardware step

The graph uses frozen peer snapshots and finite ring samples; a moving/occupied goal or narrow/impossible route can still defeat guidance. Cached edges and waypoint transitions are preferences, checked by fresh HOCBF-QPs each cycle. This is not planner completeness, universal arrival, recursive feasibility or continuous physical collision-avoidance proof. Invalid initial h/psi1, empty hard feasible sets and unresolved stalls still reject/abort.

The earlier cf3 stale onboard-pose telemetry fault is independent and is not fixed by route planning. Radio/backend/onboard telemetry continuity still needs diagnosis. The source profiles changed, so the previous survey hashes are invalid: capture and preview the actual current aircraft/obstacle geometry again before live checks or flight. No aircraft commands were sent during this development/validation.

Failed development runs remain separately labelled `archived_trial_reset_fallback`, `profiles_before_continuity_fix` and `*_before_final` and are not the final passing results.
