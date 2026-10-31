# Architecture

One-aircraft hover and six-aircraft serial swaps using high-level services. No REDCHO or HOCBF-QP controller.

Motive → NatNet → motion_capture_tracking → `/poses` → Crazyswarm2 C++ server
→ Crazyradio → onboard estimator/controller. Pose/status returns to the host.

The commissioning edition uses high-level arm/takeoff/go-to/land services.
It checks pose validity, estimator alignment, static planned path clearance and
runtime tracking. Six-aircraft serial swaps move one aircraft at a time through
a buffer. There is no REDCHO observer or HOCBF optimization in this edition.

Diagnostics preserve source/receipt clocks and algorithm session/sequence where
available. Position messages do not encode algorithm sequence or onboard receipt.
Recorders must use the exact executed profile, including surveyed geometry.
