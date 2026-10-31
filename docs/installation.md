# Installation and dependencies

Supported build platform: Ubuntu 24.04, Python 3.12, ROS 2 Jazzy. Hardware uses
Crazyflie/Crazyradio through Crazyswarm2 and OptiTrack/Motive through NatNet.
Motive runs on the lab's Windows PC; it does not need ROS.

## Offline software path

```bash
bash scripts/setup.sh offline
bash scripts/verify.sh
bash scripts/test.sh
```

Uses Ubuntu NumPy/SciPy/YAML/Matplotlib and Qt/PyQtGraph. Wrappers isolate
user-site packages and use system Python. No ROS initialization or radio access
is needed. The release was checked with NumPy 1.26.4 and SciPy 1.11.4.

## ROS/backend path

Install ROS Jazzy using the official guide linked from the root README. Then:

```bash
bash scripts/setup.sh ros
/usr/bin/python3 scripts/fetch_dependencies.py
bash scripts/resolve_dependencies.sh
bash scripts/build.sh
bash scripts/verify.sh --ros
```

The setup installs build, libusb, Boost, colcon, ROS/numerical/Qt dependencies,
and a workspace-local system-site-packages venv containing pinned rowan 1.3.2.
It does not modify shell startup files or flash firmware. The build uses this
checkout, the base Jazzy environment and explicitly selected package dependencies.

`dependencies.repos` pins the two upstream roots. `dependency-lock.json` records
16 root/submodule revisions, both patches and patched-file hashes. The fetcher
checks existing commits, applies a patch only when applicable, and refuses
missing/changed required patches. Existing mismatched checkouts are not reset.

For diagnosis:

```bash
/usr/bin/python3 scripts/fetch_dependencies.py --check
```

No submodule source or upstream binary is silently substituted with another
version. Build/install products are tied to the checkout path; rebuild after
moving a checkout. Software tests are run before making any live connection.

## USB permissions

The preserved `scripts/install_usb_rules.bash` changes udev/group configuration
and may require sudo and a login/replug. Read it, then apply once if needed:

```bash
bash scripts/install_usb_rules.bash
```

Record actual Crazyflie firmware, Crazyradio model/firmware and Motive version
alongside an experiment. No fresh firmware/peer-table validation is implied by
the source pins or a successful ROS service response.

Run resolve_dependencies.sh after fetching and before building. It uses rosdep on the selected package dependency closure, including PCL/Eigen and ROS libraries, rather than assuming those libraries already exist on a clean machine.
