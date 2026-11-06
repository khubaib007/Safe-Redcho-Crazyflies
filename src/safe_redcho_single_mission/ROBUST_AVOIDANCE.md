# Enhanced single-bank avoidance — version 0.3.0

This revision changes only `safe_redcho_single_mission`, its `single_mission*` profiles and its dedicated validation tools. The existing dual-bank controllers/profiles are unchanged. The earlier single-bank baseline is frozen, with SHA-256 hashes, in `results/checkpoints/single_position_before_avoidance_20261102/`.

The controller still has exactly one order-2 position REDCHO bank per aircraft. Battery scaling, SVFs, formation offsets, measured-state guards, command journaling, landing recovery and the separate namespace remain. The enhancements sit around the nominal tracking law and hard HOCBF-QP; no additional estimator bank was added.

The 0.2.0 state, including the user's current profiles, is preserved in `results/checkpoints/single_before_route_guidance_20261104/` with SHA-256 hashes. The complete implementation mathematics are in [MATHEMATICAL_FORMULATION.md](MATHEMATICAL_FORMULATION.md).

## What changed

### Predictive hard constraints

`barrier_domain.py` checks both geometric clearance h and the first HOCBF quantity psi1 before accepting a reference state. The QP includes additional affine constraints protecting next-step psi1 for aircraft pairs, static cylinders, walls, floor and ceiling. The completed three-agent batch is checked against the exact next state and the existing exact minimum-clearance test throughout the constant-acceleration hold.

For squared-distance barriers, next-step psi1 is convex quadratic in relative acceleration. Its tangent about the shared previous-command anchor is a global affine lower bound; enforcing that lower bound is conservative. Bounded virtual-acceleration uncertainty is reserved explicitly over the hold, in the appropriate units. None of these rows has a safety slack variable.

This adds one-step admissibility protection. It does not prove recursive feasibility under arbitrary future goals or actuation conflicts. Invalid initialization is rejected rather than reset into a fabricated safe state.

### Speed/slew anticipation

A next-step speed box alone can accept acceleration that cannot ramp down quickly enough before reaching the speed limit. With hard slew cap J, positive acceleration u must satisfy:

`dt*u + u²/(2*J) <= vmax - v`.

The negative direction is treated symmetrically. The triangular reserve conservatively bounds the remaining velocity increase while acceleration ramps to zero. This protects the speed/slew subsystem; it does not establish full multi-constraint viability. An already trapped state is rejected.

### Obstacle-aware pair responsibility

Equal pair responsibility can be infeasible when one aircraft is constrained by an obstacle or wall and another has room to yield. With `adaptive_pair_responsibility: true`, each agent computes the same bounded static-control polytopes from the common-cycle state. These include reserved static HOCBF/predictive rows, acceleration, velocity and hard slew limits. XY support minima are computed by enumerating 2D polygon vertices; no external LP solver or joint motion QP is introduced.

For a reciprocal pair inequality `A_i*u_i + A_j*u_j <= B`, let `m_i` and `m_j` be the minimum achievable contributions in those static polytopes. Bounds are allocated as:

`b_i = m_i + (B - m_i - m_j)/2`, and symmetrically for j.

Thus `b_i + b_j = B`. Existing row reserves are applied afterward. The joint safety inequality is preserved while the aircraft with greater available authority can take more avoidance effort. Empty static polytopes and insufficient combined authority are rejected. Other pair constraints are not included in these support polytopes; simultaneous multi-pair conflicts can still be infeasible.

The unequal-battery obstacle/pair conflict discovered during development is retained as a frozen regression: equal responsibility fails, while the new allocation produces three accepted commands without changing clearance or actuation bounds.

### Passing guidance and adaptive reference pacing

A safety filter can stop two aircraft without choosing a route around each other. The current local guide first tests the full segment toward the current REDCHO target. When blocked, it searches a visibility graph around the inflated static cylinders and current peer disks. Each disk contributes 24 circumscribed ring samples; candidates inside other disks or outside the wall margin are removed. Every edge is checked over its whole segment, and Dijkstra's algorithm selects the shortest available path in this finite snapshot graph. Extra planning clearance tapers at tight endpoints; hard pair/obstacle radii are never reduced.

The nominal XY velocity points toward the next waypoint and is capped at 0.13 m/s. A velocity servo supplies the XY nominal acceleration; Z tracking is unchanged. The QP still enforces acceleration, speed, slew and every safety row. The graph replans every 0.5 logical seconds or when the target changes by more than 0.10 m. A clear full direct segment releases routing and restores the original tracking law. The target is the actual current bank-plus-offset target, so FORWARD/STOP use the translated goal rather than the old acquisition endpoint. Routes can change as the target and peer snapshot change; no bank or SVF state is reset.

Passing context is retained when switching between waypoints, direct tracking and the circulation fallback, so a temporary clear segment does not erase an in-progress passing decision. A recorded near-wall case exercises this handoff explicitly.

If a peer currently occupies the goal, or a graph cannot be built within its 256-node budget, the original circulation fallback remains. It waits for persistent near-stationary target lag, then uses a latched pair chirality. Walls, obstacles and third-aircraft clearance influence that preference. Neither mechanism generates a vertical escape.

The graph freezes peer positions for route selection; this is not a dynamic collision certificate or a complete joint route planner. Cached edges can become obstructed, waypoint switching can cut a corner, and finite sampling can miss a narrow passage. Fresh hard QPs and the complete held-path validation remain the authority every cycle. Occupied goals, impossible routes and empty feasible QPs can still require abort/recovery.

The coordinator computes one next-cycle motion progress scale from the complete result batch. All three agents apply that common factor to their battery-dependent centre progress and shared shape clock. SVF and REDCHO states continue without resets. Excessive virtual target lag slows progression; active passing has its own reduced pace. The physical measured-state guards remain separate.

The stage machine detects sustained globally low measured motion away from the goal. It does not detect every individual deadlock or prove monotonic progress. With route guidance enabled, the FORWARD phase may use the configured 80 s stage budget when it exceeds the original nominal/battery-only timeout; the 12 s stall window and 180 s overall logical mission budget remain. Blocked goals or infeasible QPs fail and enter recovery; no command is published from a partial/failed batch.

## Configuration

The root `single_mission*` scenario profiles and the installed mission template enable the enhancements. Defaults in the code remain disabled for backward-compatible baseline configurations.

| Setting | Enhanced profiles |
|---|---:|
| Combined nominal formation phase | 15 s (previously 40 s) |
| HOCBF gains k1 / k2 | 2 / 2 |
| Maximum reference speed | ±0.19 m/s per axis |
| Maximum reference acceleration | ±0.4 m/s² per axis |
| Ordinary acceleration slew | 0.8 m/s³ |
| Hard safety slew cap | 8 m/s³ |
| Required virtual XY pair separation | 0.718 m |
| Measured abort XY separation | 0.6 m |
| Passing nominal speed | 0.13 m/s |
| Passing activation | lag >0.10 m and speed <0.035 m/s for 0.6 s |
| Predictive / authority allocation | enabled |
| Visibility route guidance | enabled |
| Route replan / target-change threshold | 0.5 s / 0.10 m |
| Extra route / wall planning margin | up to 0.025 m / 0.012 m |
| Circle samples / maximum graph nodes | 24 / 256 |
| Nonfinal waypoint arrival tolerance | 0.08 m |

The 15 s phase is a request, not a guaranteed completion time. Adaptive pacing, QP intervention and settling extend it when needed. No physical separation, uncertainty reserve, obstacle inflation, speed limit, acceleration limit or hard safety slew cap was reduced. All enhanced scenarios explicitly enable the existing safety-priority slew policy.

`avoidance` options are validated strictly. Remove neither the hard checks nor the source/hash guards to make a failed preview pass. Edited source profiles invalidate existing surveys: **capture again before starting backend/agents/check/flight.**

For an architecture-only single-versus-dual-bank ablation, use the frozen pre-enhancement baseline or restore its gains/durations/slew settings and disable `enabled`, `predictive_barriers` and `adaptive_pair_responsibility`. The enhanced controller changes more than the estimator architecture and must be compared accordingly.

## Telemetry and diagnosis

- `avoidance`: selected hazards/side, activation, blend, nominal correction and target lag.
- `avoidance.route`: search status/reason, waypoint, route length, node count and search wall time; blocked snapshots retain the existing fallback.
- `governor.motion_progress_scale`: actual common progress factor.
- `qp.rows.next_*`: additional predictive rows, normalized residuals and current geometry; `row_family` identifies their role.
- `qp.pair_responsibility`: policy, support minima, combined slack and allocated bounds with native units.
- `qp.constraint_wall_s`: constraint/authority construction time, separate from solver time.
- Top-level filter timing includes total filter work; the ROS cycle deadline still covers the complete computation/transport path.

The control plot includes passing activation/blend and motion-progress panels. Existing position-bank derivative, HOCBF, measured flight, battery and timing views remain. Solver clipping warnings concern internal SLSQP iterates; every returned command is independently checked against all final hard rows/bounds and the batch checks.

## Reproduce software validation

Rebuild the separate baseline overlay after source changes using the README build command. Run the unit/integration tests:

```bash
cd ~/safe_redcho_hardware
PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 \
PYTHONPATH=src/safe_redcho_single_mission QT_QPA_PLATFORM=offscreen \
  python3 -s -m pytest src/safe_redcho_single_mission/test -q -p no:cacheprovider
```

Run the complete numerical matrix (all four scenarios at nominal/unequal voltages, plus a 7.5 s combined-phase stress request):

```bash
PYTHONNOUSERSITE=1 PYTHONPATH=src/safe_redcho_single_mission \
  python3 -s src/safe_redcho_single_mission/test/validate_scenarios.py \
  --workspace "$PWD" --output results/single_avoidance_manual
```

`results/single_route_validation_20261104/` contains the current route-regression reports, traces, test results and transport evidence. `results/single_avoidance_final_20261102/` records the preceding 0.2.0 implementation. Earlier `single_avoidance_trials_*` and `single_avoidance_validation_*` directories include development failures and are not the current qualification matrix.

The encounter regressions exercise actual per-agent QP solves and complete batch checks each cycle: head-on swaps, perpendicular crossings, cylinder obstruction and near-wall passing. Four cases use a 4 m synthetic arena; a separate near-wall swap uses a 3 m arena. These are explicitly labelled test geometries, not changes to the calibrated hardware room. Unsafe initialization, insufficient authority and conflicting hard constraints are also tested for rejection.

No aircraft commands were sent during this work. Numerical and synthetic ROS results do not validate physical tracking, radio delivery or recovery on this controller. Recommission through a new survey, captured preview, live ground-only check and operator-supervised acquisition before full flight. Static cylinders remain the obstacle model; moving obstacles invalidate that model and trigger the existing guard. General route completion and universal collision avoidance are not claimed.
