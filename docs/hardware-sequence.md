# Hardware command sequence

Complete offline tests, dependency/build verification, network/USB configuration
and actual config/local edits first. Fit props only after ground axis/estimator
checks. Commands below are operator actions; they were not run during publishing.
Always use the same checkout and scenario in every terminal.

## Default mission

1. Terminal A: `bash scripts/run.sh mocap`.
2. Terminal B: `bash scripts/run.sh capture` while aircraft are stationary on floor.
3. Stop standalone mocap. Terminal A: `bash scripts/run.sh backend`.
4. Terminal B: `bash scripts/run.sh agents`.
5. Terminal C: `bash scripts/run.sh observe`.
6. Terminal D: `bash scripts/run.sh check` for live ground checks and preview.
7. `bash scripts/run.sh acquire` physically acquires formation and lands.
8. `bash scripts/run.sh fly` physically performs the complete mission and lands.

## Surveyed combined experiment

Pass `--scenario combined` in **every** command:

```bash
# Terminal A, pose-only survey receiver
bash scripts/run.sh mocap --scenario combined

# Terminal B, all aircraft grounded; detects aircraft starts and obstacle poses
bash scripts/run.sh capture --scenario combined
```

Stop standalone mocap, then start separate terminals:

```bash
# Terminal A
bash scripts/run.sh backend --scenario combined
# Terminal B
bash scripts/run.sh agents --scenario combined
# Terminal C
bash scripts/run.sh observe --scenario combined
# Terminal D: live ground checks, no arm/takeoff
bash scripts/run.sh check --scenario combined
```

After the live checks and physical commissioning are satisfactory, the operator
may run `bash scripts/run.sh acquire --scenario combined`, then a separately
requested full run with `bash scripts/run.sh fly --scenario combined`.

Use obstacle or interagent instead of combined for those profiles. Each scenario
needs its own survey. Live commands resolve the same immutable profile; observe
generates telemetry/plot configs bound to that exact profile. A changed source
requires a new survey while control processes are stopped and aircraft grounded.

Ground checks do not arm/take off or publish flight position commands. The
existing mission may update optional status LEDs through backend parameters.

For the applicable offline scenario use run.sh offline; it never replaces live
capture. After verified landing/disarm, stop observers, agents/watchdog and backend
in that order. See recovery.md for failures and deliberate emergency procedures.
