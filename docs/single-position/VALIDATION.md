# Snapshot provenance and validation limits

Release: `single-position-v0.3.0`, prepared from the current hardware workspace
on 6 November 2026. The package source is the restored 0.3.0 controller with the
separately requested `node_shutdown.py` and duplicate-backend startup/diagnostic
patch. No safety bound was loosened by publication.

`validation/single_source_snapshot.json` records source-workspace file hashes.
All package Python source is copied unchanged. Portable profile examples replace
the old absolute workspace prefix with relative paths; the controller's numeric
parameters remain unchanged. `scripts/single_run.py` adapts paths and launch
selection for this repository, uses ignored local configuration and calls the
same scoped shutdown supervisor. Its configuration/command tests do not move
aircraft.

Previously recorded evidence, included with explicit scope:

- `single_rollback_summary.json`: restored 0.3.0 software checks and a complete
  3130-cycle synthetic combined mission.
- `single_combined_offline_report.json`: the numerical mission's separation,
  endpoint and QP measurements.
- `single_backend_summary.json`, `single_backend_tests.log` and
  `single_backend_inert_dds.json`: 42 focused tests and actual isolated DDS
  ownership tests for the startup patch. The obsolete backend was stopped only
  after the user explicitly confirmed grounded/disarmed state.
- `historical-route-validation.md`: earlier controller validation history. Raw
  datasets and checkpoint directories mentioned there are not bundled.

Publication checks are recorded separately in
`validation/single_publication_validation.json`. Software/DDS/numerical results
do not validate firmware timing, radio packet receipt, actuator authority or
physical tracking. A successful clean physical flight of this final snapshot
is not claimed. Run fresh capture, preview, ground check and acquisition
commissioning on the actual lab setup.

This is a usable single-bank baseline, but an architecture-only ablation still
requires matched gains, safety settings, measured geometry, battery inputs and
timing in the dual-bank comparison.
