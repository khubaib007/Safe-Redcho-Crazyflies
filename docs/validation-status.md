# Validation status: mission-v0.1.0

Release preparation date: 31 October 2026, Asia/Riyadh.

| Tier | Scope |
|---|---|
| Static checks | Python/XML/Bash syntax and numerical imports |
| Unit/adaptor tests | Pure logic and explicit ROS/transport doubles |
| Offline run | Synthetic geometry/reference checks; see included reports |
| Backend build | Pinned upstream source plus both required local patches |
| Edition build | First-party packages against that freshly built backend |
| Interface check | Actual generated classes and executables; no ROS initialization |
| Live RF/mocap/firmware/flight | Not performed during this publishing task |

Detailed completed results are appended before publication. Software reports
are not physical flight evidence. Neither a ROS service reply nor a plot alone
proves onboard receipt or execution. The six-agent FullState edition and each
position-command edition require their own hardware commissioning.

Any older flight reports explicitly labeled historical apply to the original
workspace/configuration at that time, not automatically to this prepared snapshot.

## Completed software verification

- 180 unit/adaptor/GUI/config tests ran; two original bundle installer tests skipped; all other tests passed.
- Offline scenarios passed: offline_interagent, offline_combined, offline_default, offline_obstacle.
- Six pinned upstream backend/interface packages built from exported exact commits plus both local patches.
- All 2 edition packages built against that fresh backend.
- Actual ROS interfaces and binaries verified without initializing a node.
- Dependency fetcher patch application and idempotent checks passed using clean local Git clones at every pinned revision.
- No live radio, Motive stream, firmware readback or physical flight was performed.

Small reports are in validation/. Full software logs/traces are attached as a release validation archive. The GitHub offline workflow result is separate remote evidence; the manual backend-build workflow is provided for clean-machine repetition.
