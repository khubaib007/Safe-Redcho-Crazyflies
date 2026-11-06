# Safe REDCHO Crazyflies — single position bank

**Edition:** `single-position-v0.3.0` · **ROS package:** `safe_redcho_single_mission` · **Aircraft:** cf2, cf3, cf4

Battery-aware formation acquisition and transport with one order-2 **position
REDCHO bank per aircraft**, derivative-channel velocity/acceleration estimates,
visibility guidance and local hard **HOCBF-QPs**. The hardware receives position
plus yaw. This is the restored 0.3.0 baseline with scoped Ctrl+C cleanup and the
verified duplicate-backend startup fix; publication does not relax safety bounds.

## Get started

```bash
git clone --branch single-position-v0.3.0 --depth 1 \
  https://github.com/khubaib007/Safe-Redcho-Crazyflies.git safe-redcho-single-position
cd safe-redcho-single-position
bash scripts/setup.sh offline
bash scripts/verify.sh
bash scripts/test.sh
bash scripts/redcho_single_mission.sh combined offline
```

**[Complete installation, capture, preview, checks, telemetry, flight and troubleshooting guide](docs/single-position/START_HERE.md)**

- [Single-bank architecture](src/safe_redcho_single_mission/README.md)
- [Detailed mathematical formulation](src/safe_redcho_single_mission/MATHEMATICAL_FORMULATION.md)
- [HOCBF-QP and routing design](src/safe_redcho_single_mission/ROBUST_AVOIDANCE.md)
- [Validation scope and provenance](docs/single-position/VALIDATION.md)
- [Why duplicate backends blocked readiness](docs/single-position/backend-resolution.md)

The repository remains private. Each edition has its own branch/tag; earlier
releases remain unchanged. This checkout retains existing first-party packages
and adds the single-bank package separately in `src/`. Live settings belong in
ignored `config/local/`; no live captured starts or generated profiles are shipped.

| Edition | Purpose | Release |
|---|---|---|
| Commissioning | Pose/estimator checks, hover and serial swaps | [commissioning-v0.1.0](https://github.com/khubaib007/Safe-Redcho-Crazyflies/releases/tag/commissioning-v0.1.0) |
| Original flight | Six-aircraft implementation | [flight-v0.1.0](https://github.com/khubaib007/Safe-Redcho-Crazyflies/releases/tag/flight-v0.1.0) |
| Three-aircraft | Earlier three-agent adaptation | [three-v0.1.0](https://github.com/khubaib007/Safe-Redcho-Crazyflies/releases/tag/three-v0.1.0) |
| Dual-bank mission | Formation acquisition and surveyed obstacles | [mission-v0.1.0](https://github.com/khubaib007/Safe-Redcho-Crazyflies/releases/tag/mission-v0.1.0) |
| Single position bank | Current single-bank hardware baseline | [single-position-v0.3.0](https://github.com/khubaib007/Safe-Redcho-Crazyflies/releases/tag/single-position-v0.3.0) |

Offline and isolated DDS evidence is included. A clean physical flight of this
final snapshot is not claimed: new hardware installations require actual lab
configuration, capture, preview, ground checks and acquisition commissioning.

Source is MIT licensed; fetched dependencies retain upstream licenses. The
pinned dependency lock and backend patches are inherited from the hardware
repository. See `release.json` for the selected edition and package list.

Companion: [Safe REDCHO Gazebo Simulation](https://github.com/khubaib007/Safe-Redcho-Gazebo-Simulation).
