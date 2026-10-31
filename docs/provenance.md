# Provenance: flight-v0.1.0

Prepared from the existing hardware workspace on 31 October 2026. These are
standalone source snapshots, not recreated historical Git commits. The active
working workspaces and prior experiments were not edited or flown during release
preparation. Initial source hashes are in validation/source_snapshot.json.

The exact local Crazyswarm2 and crazyflie_cpp changes are captured as two patches;
dependency-lock.json pins roots/submodules and patch/file hashes. Existing
upstream backup files and the stray untracked motion-capture rclpy file are not
runtime dependencies and are not imported. Full vendored Git trees are fetched,
not copied into source commits.

Preparation changes:

- Add portable entry points, local-config initialization, explicit tests, guides,
  source manifests and offline CI. Preserve original wrappers as historical notes.
- Adapt bringup configuration/source audit root detection for config/local.
- Replace test dependence on mutable lab values with explicit deterministic
  fixtures and current transport field names. Preserve actual live guard values.
- Fix legacy live-plot pen-style enum access in the three edition.
- In the mission edition, validate positive arena/backend fields, fail final
  status on journal-drain errors, and bind observation to the executed survey
  profile. Focused regression checks cover those changes.
- Normalize published example paths and remove real captured-start files.

Root configuration is an example. Offline traces are synthetic; build/interface
checks are separate from physical commissioning. Historical source notes under
docs/history do not replace the current guides or validation-status.md.
