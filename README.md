# Safe REDCHO Three-Aircraft Position Control

Three-aircraft dual-REDCHO/HOCBF-QP position control with a prechecked slow handover.

**Edition:** `three-v0.1.0` · **Aircraft:** Three: cf2, cf3, cf4 · **Interface:** Position and yaw

The repository starts private for review. Each tag contains a complete edition
and guides. The current checkout's controller is identified in `release.json`.

| Edition | Purpose | Release |
|---|---|---|
| Hardware Bringup | Pose/estimator checks, hover, serial swaps | [commissioning](https://github.com/khubaib007/safe-redcho-hardware/releases/tag/commissioning-v0.1.0) |
| Six-Aircraft Full-State | Original six-agent REDCHO/HOCBF reference control | [flight](https://github.com/khubaib007/safe-redcho-hardware/releases/tag/flight-v0.1.0) |
| Three-Aircraft Position Control | Three-agent adaptation and slow handover | [three](https://github.com/khubaib007/safe-redcho-hardware/releases/tag/three-v0.1.0) |
| Formation Acquisition | Current mission and surveyed obstacle experiments | [mission](https://github.com/khubaib007/safe-redcho-hardware/releases/tag/mission-v0.1.0) |

## Start offline: no ROS or aircraft needed

Ubuntu 24.04 / Python 3.12:

```bash
git clone --branch three-v0.1.0 --depth 1 https://github.com/khubaib007/safe-redcho-hardware.git safe-redcho-hw-three
cd safe-redcho-hw-three
bash scripts/setup.sh offline
bash scripts/verify.sh
bash scripts/test.sh
bash scripts/run.sh offline --output results/first_offline_run
```

Expected: the test report and offline report say PASS. Test suites use explicit
ROS doubles or pure numerical code. They do not connect to radios or arm aircraft.
The numerical model screens references/geometry; it does not validate firmware,
RF delivery or physical tracking. See [validation status](docs/validation-status.md).

## Prepare a hardware installation

First install [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).
From a clean terminal in this checkout:

```bash
bash scripts/setup.sh ros
/usr/bin/python3 scripts/fetch_dependencies.py
bash scripts/resolve_dependencies.sh
bash scripts/build.sh
bash scripts/verify.sh --ros
bash scripts/run.sh init
```

Dependencies are fetched at exact commits, including submodules. Two captured
local backend/radio-library patches are applied and verified. No dependency is
automatically pulled to a newer version. `verify.sh --ros` imports actual
interfaces and locates executables without creating nodes or radio connections.

Edit **`config/local/`** for your lab: aircraft URIs, network, measured floor and
arena, and active profile. This directory is ignored by Git. No real captured
starts are supplied or fabricated by initialization. Follow the edition-specific
[hardware sequence](docs/hardware-sequence.md) to capture actual grounded starts.

## Guides

- [Installation, dependencies and USB access](docs/installation.md)
- [Algorithm and command architecture](docs/architecture.md)
- [Configuration and measured starts](docs/configuration.md)
- [Numbered hardware command sequence](docs/hardware-sequence.md)
- [Verification and tests](docs/testing.md)
- [Troubleshooting](docs/troubleshooting.md)
- [Landing and recovery](docs/recovery.md)
- [Validation evidence and limits](docs/validation-status.md)
- [Provenance and release changes](docs/provenance.md)

```text
src/                  Edition's first-party packages
config/*.yaml         Examples; not calibrated settings for your aircraft
config/local/         Your editable lab settings and captured starts (ignored)
scripts/              Setup, fetch, build, verify, test and run commands
patches/dependencies/ Exact local changes against pinned upstream commits
dependency-lock.json  Parent/submodule revisions and patch/file hashes
validation/           Software/build reports and source manifests
docs/                 Starting guide, architecture, troubleshooting and recovery
```

Original working workspaces and historical results are preserved separately.
First-party code is [MIT licensed](LICENSE); fetched dependencies retain their
own upstream license files. No full experiment datasets, virtual environments,
editor indexes, radio credentials or generated build products are source commits.

Companion project: [Safe REDCHO simulation](https://github.com/khubaib007/safe-redcho-simulation). Its native motor-level simulation validation is separate from hardware commissioning.
