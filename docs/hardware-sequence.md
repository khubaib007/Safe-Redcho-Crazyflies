# Hardware command sequence

Complete offline tests, dependency/build verification, network/USB configuration
and actual config/local edits first. Fit props only after ground axis/estimator
checks. Commands below are operator actions; they were not run during publishing.
Always use the same checkout and scenario in every terminal.

## One aircraft first

1. Terminal A: `bash scripts/run.sh mocap`. Inspect physical axes and reception.
2. Stop standalone mocap. Terminal A: `bash scripts/run.sh single-backend`.
3. Terminal B: `bash scripts/run.sh single-check`. No arming/takeoff is requested.
4. With the live checks passed and operator ready: `bash scripts/run.sh single-hover`.
   This arms/takes off and performs the short hover/landing sequence.

## Six-aircraft commissioning

1. Terminal A: `bash scripts/run.sh mocap` with the six grounded bodies visible.
2. Terminal B: `bash scripts/run.sh capture` to record actual starting positions.
3. Stop standalone mocap; Terminal A: `bash scripts/run.sh backend`.
4. Terminal B: `bash scripts/run.sh check` for monitored ground checks.
5. `bash scripts/run.sh fleet-hover` performs physical takeoff, hover and landing.
6. After rechecking the actual layout and complete swap plan, `bash scripts/run.sh swap`
   performs the seven serial go-to legs and landing.

Swap geometry with actual starts must pass, including full path margins and the
flight-duration budget. The offline synthetic hexagon is not a floor-placement
instruction or a captured-start substitute.

For the applicable offline scenario use run.sh offline; it never replaces live
capture. After verified landing/disarm, stop observers, agents/watchdog and backend
in that order. See recovery.md for failures and deliberate emergency procedures.
