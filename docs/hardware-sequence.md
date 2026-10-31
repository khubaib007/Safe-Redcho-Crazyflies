# Hardware command sequence

Complete offline tests, dependency/build verification, network/USB configuration
and actual config/local edits first. Fit props only after ground axis/estimator
checks. Commands below are operator actions; they were not run during publishing.
Always use the same checkout and scenario in every terminal.

1. Terminal A: `bash scripts/run.sh mocap`.
2. Terminal B: `bash scripts/run.sh capture` with actual grounded aircraft.
3. Stop standalone mocap. Terminal A: `bash scripts/run.sh backend`.
4. Terminal B: `bash scripts/run.sh agents`; keep agents/watchdog alive.
5. Terminal C: `bash scripts/run.sh observe` for recorders.
6. Terminal D: `bash scripts/run.sh check`; no arm/takeoff is requested.
7. `bash scripts/run.sh hover` performs physical streamed hover commissioning.
8. After successful commissioning: `bash scripts/run.sh fly` performs translation
   and recovery/landing. This is a physical-flight command.

The six-aircraft edition sends full-state references. Verify actual firmware and
controller support before commissioning that interface. It is preserved as a
distinct legacy edition, not automatically promoted to the newest position stack.

For the applicable offline scenario use run.sh offline; it never replaces live
capture. After verified landing/disarm, stop observers, agents/watchdog and backend
in that order. See recovery.md for failures and deliberate emergency procedures.
