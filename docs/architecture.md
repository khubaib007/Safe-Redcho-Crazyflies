# Architecture

Original six-aircraft distributed dual-REDCHO reference governor with reciprocal HOCBF-QP.

Motive → NatNet → motion_capture_tracking → `/poses` → Crazyswarm2 C++ server
→ Crazyradio → onboard estimator/controller. Pose/status returns to the host.

Each agent owns an SVF, a velocity REDCHO bank, a position/height REDCHO bank
and a local reciprocal HOCBF-QP. Measured battery voltage advances the local
reference phase; no simulated state-of-charge is substituted for voltage.
Two ring neighbors exchange same-cycle observer estimates. A coordinator sends
common virtual-reference state and voltages and accepts only complete command
batches. A separate watchdog monitors heartbeats and requests recovery.

```mermaid
flowchart LR
    V[Measured battery voltage] --> G[Reference governor and SVF]
    G --> R[Velocity and position REDCHO]
    R --> C[Nominal reference tracking]
    C --> Q[Local reciprocal HOCBF-QP]
    Q --> I[Validated virtual-reference integration]
    I --> B[Crazyswarm2 command adapter]
    B --> F[Onboard control and actual aircraft]
    F --> H[Measured pose and tracking guards]
    H --> I
```

The HOCBF acts on a virtual reference plant with inflated pair/arena/obstacle
allowances. Physical tracking is monitored separately. A safe virtual reference
does not establish arbitrary disturbed-aircraft safety. These hardware stacks
do not compute or send the four motor forces used in the simulation repository.

This edition sends FullState messages containing reference position, orientation, velocity and acceleration. It uses six agents and ALIGN/FORWARD/STOP phases. FullState support and onboard behavior must be commissioned with the actual firmware; this publication does not establish fresh RF/firmware execution.

Diagnostics preserve source/receipt clocks and algorithm session/sequence where
available. Position messages do not encode algorithm sequence or onboard receipt.
Recorders must use the exact executed profile, including surveyed geometry.
