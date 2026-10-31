# Configuration: three-v0.1.0

Initialize editable files once:

```bash
bash scripts/run.sh init
```

This creates `config/local/` without overwriting existing files. Edit the lab
settings there. Root `config/*.yaml` files are portable examples and numerical
fixtures. Their old lab dimensions, URI examples and thresholds are not a new
room calibration or a recommendation to change a working aircraft address.

## Required physical measurements

- `lab_config.yaml`: Motive/Ubuntu wired IP addresses, mocap backend/rate/frame,
  aircraft rigid-body name, actual URI and measured initial position.
- Fleet file: exact unique URIs and radio channels. The examples contain a
  three-dongle layout; use your measured/configured hardware routing.
- Arena: floor origin, X/Y limits, ceiling, body/marker envelope and allowances.
- Controller profile: active names, geometry, reference trajectory and accepted
  tracking limits. Preserve units and guard ordering.

Example NatNet configuration is multicast 239.255.42.99 on ports 1510/1511,
world frame with Z up and metres. Verify actual received axes and rigid-body
pivot by hand before attaching props. A frame label alone does not rotate data.
The body name must match the aircraft name exactly. Keep exactly one `/poses`
publisher; stop standalone mocap before starting a backend that includes mocap.

Check routes using `ip route get <MOTIVE_IP>` and `ip route get 239.255.42.99`.
The latter should select the wired interface. The preserved network_check.bash
is a read-only aid. Narrow multicast/interface changes belong to the lab network
setup, not to a flight script.

Captured starts are created by the edition's capture command while aircraft are
stationary and grounded. Initialization deliberately creates no measured-start
file. Changing configured initial_position does not move the physical aircraft.

## Edition's profile

Use `config/local/redcho_three.yaml`; only cf2/cf3/cf4 are active. The fleet inventory still contains six entries for URI validation, while the backend is restricted to the active three. The handover geometry is checked before arming.

Do not edit generated backend configurations. They are overwritten. Do not mix
nodes from different editions or overlays. Domain 43 separates the documented
hardware ROS graph from the simulation default, but cannot prevent two programs
from addressing the same radio URI.
