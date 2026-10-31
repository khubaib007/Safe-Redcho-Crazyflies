# Verification and tests

```bash
bash scripts/verify.sh
bash scripts/test.sh
```

The suite saves logs and test_report.json under a new results/tests_<UTC>/ folder.
Every subprocess must exit zero. Tests use pure geometry/numerics or explicit
ROS/transport doubles; they do not initialize a live flight graph. The mission
suite includes its real Qt windows offscreen, source-timestamp checks, obstacle
guards, QP priority, survey consistency, selective recovery and no-arm checks.
The three-aircraft suite also exercises the actual slow-handover functions.

Offline profiles and captured-start fixtures are synthetic and versioned. Tests
must remain independent of config/local and actual lab captures. No safety
threshold in live configuration was relaxed to make historical tests pass.

After dependency fetch/build:

```bash
bash scripts/verify.sh --ros
```

This imports actual generated message/service classes and locates backend
executables. It creates no ROS nodes or radio connections. A successful build
does not establish mocap compatibility, RF delivery, onboard settings or flight.

The GitHub Actions offline workflow runs software tests and saves artifacts.
Physical commissioning is never started by CI. See validation-status.md for the
precise build, offline and historical evidence included in this release.

A separate manual build workflow fetches pinned dependencies, resolves system libraries, builds, and imports interfaces. It does not start backend nodes, mocap or aircraft operations. Its GitHub runner execution status is independent of the recorded local build checks.
