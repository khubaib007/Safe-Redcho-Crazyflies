# Landing and recovery

Keep the backend and independent watchdog running during controlled recovery.
Do not stop the radio/mocap backend while airborne. The mission's recovery
requests release of streamed commands where required, requests landing, and
disarms only aircraft with qualifying recent touchdown evidence.

A ROS service reply establishes a host request/reply, not physical RF delivery.
If a landing cannot be verified, use the lab's independent operator recovery
procedure. Establish and test that procedure with props removed before flight.

For a deliberately required motor-cut emergency, in a separately sourced
terminal, address the actual active aircraft individually, for example:

```bash
source scripts/environment.bash
source scripts/ros_environment.bash
ros2 service call /cf2/emergency std_srvs/srv/Empty '{}'
```

Emergency cuts motors; it is not a controlled landing. Select the appropriate
active aircraft and use only when that action is required. This is an operator
command, never part of automated release tests. It still depends on the backend,
radio and aircraft being able to receive it.

After verified touchdown/disarm, stop observation first to drain files, then
agents/watchdog and backend. Preserve status, raw logs and partial traces after
an abort; do not classify an interrupted/uncertain mission as a completed flight.
