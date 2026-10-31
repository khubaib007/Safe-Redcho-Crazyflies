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

Only cf2/cf3/cf4 are active. The pre-arm handover geometry must pass. The
slow position bridge settles before REDCHO ALIGN/FORWARD begins. Do not start a
six-aircraft backend or other command publishers at the same time.

For the applicable offline scenario use run.sh offline; it never replaces live
capture. After verified landing/disarm, stop observers, agents/watchdog and backend
in that order. See recovery.md for failures and deliberate emergency procedures.
