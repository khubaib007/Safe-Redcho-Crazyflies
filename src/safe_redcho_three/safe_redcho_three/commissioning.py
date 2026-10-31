"""Selective command routes and a slow position-only hover handover."""
import csv
import json
import time
import numpy as np
from .fleet_adapter import ACTIVE, INACTIVE, require_active
from .settings import slots, pair_distance, virtual_box


def setup_commissioning(node):
    from crazyflie_interfaces.srv import Arm, Takeoff
    require_active(node.names)
    node.inactive_probe_clients = {n: {
        'arm': node.create_client(Arm, f'/{n}/arm'),
        'takeoff': node.create_client(Takeoff, f'/{n}/takeoff'),
    } for n in INACTIVE}
    node.get_logger().info('ACTIVE AIRCRAFT: cf2, cf3, cf4 ONLY; streamed position commissioning')


def authorize_requests(label, requests, names):
    require_active(names)
    if set(requests)-set(ACTIVE):
        raise RuntimeError('Inactive aircraft in '+label+' requests')
    if label not in ('arm', 'takeoff', 'land', 'notify_setpoints_stop'):
        raise RuntimeError('Unknown flight service '+label)
    if label == 'arm' and any(bool(r.arm) for r in requests.values()):
        if set(requests) != set(ACTIVE) or not all(bool(r.arm) for r in requests.values()):
            raise RuntimeError('Arming must select exactly cf2, cf3, cf4')


def check_command_routes(node):
    require_active(node.names)
    for name in INACTIVE:
        if any(c.service_is_ready() for c in node.inactive_probe_clients[name].values()):
            raise RuntimeError('Inactive aircraft service exists: stop the six-aircraft backend')
    for name in ACTIVE:
        for topic in ('cmd_full_state', 'cmd_vel_legacy', 'cmd_hover', 'cmd_velocity_world'):
            if node.count_publishers('/'+name+'/'+topic):
                raise RuntimeError('Competing flight command publisher: /'+name+'/'+topic)


def publish_positions(node, q, v, u):
    from crazyflie_interfaces.msg import Position
    require_active(node.names)
    q, v, u = (np.asarray(x, float) for x in (q, v, u))
    if any(x.shape != (3, 3) or not np.isfinite(x).all() for x in (q, v, u)):
        raise RuntimeError('Invalid three-aircraft reference batch')
    if set(node.fullstate_publishers) != set(ACTIVE):
        raise RuntimeError('Unexpected command publisher route')
    started = time.monotonic()
    if not hasattr(node, '_command_file'):
        node._command_file = (node.output/'position_commands.csv').open('w', newline='', buffering=1)
        node._command_csv = csv.writer(node._command_file)
        node._command_csv.writerow(['wall_monotonic_s', 'ros_ns', 'stage', 'name',
            'published_x_m', 'published_y_m', 'published_z_m', 'published_yaw_rad',
            'computed_vx_mps', 'computed_vy_mps', 'computed_vz_mps',
            'computed_ax_mps2', 'computed_ay_mps2', 'computed_az_mps2',
            'interface', 'received_onboard_verified'])
    for i, name in enumerate(ACTIVE):
        msg = Position()
        now = node.get_clock().now()
        msg.header.frame_id = node.profile['frame_id']
        msg.header.stamp = now.to_msg()
        msg.x, msg.y, msg.z = map(float, q[i])
        msg.yaw = 0.0
        node.fullstate_publishers[name].publish(msg)
        # v and a describe the virtual reference. Position messages do NOT carry them.
        node._command_csv.writerow([time.monotonic(), now.nanoseconds, node.stage_name,
            name, *q[i], 0.0, *v[i], *u[i], 'cmd_position', False])
    return time.monotonic()-started


def bridge_clear(start, goal, cfg):
    """All drones use the same smooth interpolation fraction, so pair paths are linear in it."""
    start, goal = np.asarray(start, float), np.asarray(goal, float)
    if (start.shape != (3, 3) or goal.shape != (3, 3)
            or not np.isfinite(start).all() or not np.isfinite(goal).all()):
        raise RuntimeError('Handover requires finite three-aircraft reference endpoints')
    lo, hi = virtual_box(cfg)
    if np.any(start <= lo) or np.any(start >= hi) or np.any(goal <= lo) or np.any(goal >= hi):
        raise RuntimeError('Hover handover endpoint outside the inflated flight volume')
    for i in range(3):
        for j in range(i):
            r = start[i, :2]-start[j, :2]
            d = (goal[i, :2]-goal[j, :2])-r
            fraction = float(np.clip(-r@d/max(float(d@d), 1e-15), 0., 1.))
            separation = float(np.linalg.norm(r + fraction*d))
            required = float(pair_distance(cfg))
            if separation < required:
                raise RuntimeError(
                    f'Hover handover lacks full reference-pair clearance: '
                    f'{cfg["names"][j]}–{cfg["names"][i]}, '
                    f'minimum={separation:.4f} m, required={required:.4f} m'
                )
    if cfg['obstacles']:
        raise RuntimeError('Three-aircraft commissioning requires an obstacle-free arena')


def prepare_handover(node):
    """Certify a virtual bridge on the ground, before any arm/takeoff request.

    Takeoff has no XY-slot argument. Its planned hover endpoint is above the
    current floor XY, at the same absolute Z requested by Mission.takeoff.
    Measured aircraft may differ from this virtual reference ONLY within the
    existing tracking tube. Never use an unsafe measured batch as a reference.
    """
    cfg = node.profile
    ground, _, _ = node.health(ground=True)
    reference = np.asarray(ground, float).copy()
    goal = slots(cfg)
    if reference.shape != goal.shape or not np.isfinite(reference).all():
        raise RuntimeError('Invalid live ground positions for handover planning')
    reference[:, 2] = goal[:, 2]
    bridge_clear(reference, goal, cfg)
    duration = float(cfg['commissioning']['handover_duration_s'])
    peak = 1.875 * float(np.max(np.linalg.norm(goal-reference, axis=1))) / duration
    if peak > .04:
        raise RuntimeError('Handover reference speed would exceed 0.04 m/s')
    node.takeoff_hover_reference = reference.copy()
    node.result['handover_plan'] = {
        'status': 'PASS_BEFORE_ARM',
        'reference_start_source': 'live_ground_xy_and_requested_takeoff_z',
        'ground_position_m': np.asarray(ground, float).tolist(),
        'reference_start_m': reference.tolist(), 'slot_position_m': goal.tolist(),
        'required_reference_xy_separation_m': float(pair_distance(cfg)),
        'peak_bridge_reference_speed_mps': peak,
        'scope': 'Host geometric plan; not onboard setpoint readback',
    }
    node.get_logger().info(
        'HANDOVER_PLAN_PASS: live floor XY -> existing hover slots; '
        'full reference clearance validated before arm')


def bridge_sample(start, goal, elapsed, duration):
    s = float(np.clip(elapsed/duration, 0., 1.))
    b = s**3*(10-15*s+6*s*s)
    bd = 30*s*s*(1-s)**2/duration
    bdd = 60*s*(1-3*s+2*s*s)/duration**2
    d = goal-start
    return start+b*d, bd*d, bdd*d


def handover(node):
    """Planned takeoff hover -> certified position bridge -> settled hold."""
    cfg, dt = node.profile, node.profile['control']['dt_s']
    goal = slots(cfg)
    if not hasattr(node, 'takeoff_hover_reference'):
        raise RuntimeError('Missing pre-arm handover plan; do not plan from an airborne measurement')
    start = np.asarray(node.takeoff_hover_reference, float).copy()
    bridge_clear(start, goal, cfg)
    node.stage_name = 'HANDOVER_WAIT'
    node.expected = start.copy()

    deadline = time.monotonic() + float(
        cfg['commissioning']['settle_timeout_s']
    )
    good_since = None
    last_notice = -float('inf')
    reason = 'Waiting for a valid handover start'

    while time.monotonic() < deadline:
        measured, velocity, _ = node.health(airborne=True)
        now = time.monotonic()
        reason = None

        if np.max(np.linalg.norm(velocity, axis=1)) >= .10:
            reason = 'Hover must settle below 0.10 m/s before handover'
        elif np.max(np.linalg.norm(measured-start, axis=1)) >= .10:
            reason = 'Hover must lie within 0.10 m of its planned takeoff reference'

        if reason is None:
            good_since = now if good_since is None else good_since
            if now-good_since >= .5:
                break
        else:
            good_since = None
            if now-last_notice >= 1.:
                node.get_logger().info('HANDOVER_WAIT: ' + reason)
                last_notice = now

        node.pulse()
        node.spin_for(dt)
    else:
        raise RuntimeError(
            'No continuously valid handover start within '
            f'{cfg["commissioning"]["settle_timeout_s"]:.1f} s: '
            f'{reason or "valid-start dwell incomplete"}'
        )
    
    duration = float(cfg['commissioning']['handover_duration_s'])
    distance = np.max(np.linalg.norm(goal-start, axis=1))
    if 1.875*distance/duration > .04:
        raise RuntimeError('Handover reference speed would exceed 0.04 m/s')
    node.stage_name = 'POSITION_HANDOVER'
    initial_measured_error = np.linalg.norm(measured-start, axis=1)
    node.streaming, node.expected = True, start.copy()
    node.pulse()
    # Start from the pre-arm certified takeoff reference, NOT a possibly
    # clearance-deficient measured batch. No REDCHO/QP runs in this bridge.
    last_publish = time.monotonic()
    span = node.publish_setpoints(start, np.zeros_like(start), np.zeros_like(start))
    if span > dt:
        raise RuntimeError('Local publication of handover batch exceeded one control period')
    begun = time.monotonic()
    final_since = None
    deadline = begun+duration+float(cfg['commissioning']['settle_timeout_s'])
    while time.monotonic() < deadline:
        cycle_start = time.monotonic()
        node.health(airborne=True)
        publication_start = time.monotonic()
        elapsed = publication_start-begun
        q, v, a = bridge_sample(start, goal, elapsed, duration)
        gap = publication_start-last_publish
        if gap > cfg['control']['maximum_publish_gap_s']:
            raise RuntimeError(
                f'Position handover publication gap {gap:.6f}s exceeds '
                f'{cfg["control"]["maximum_publish_gap_s"]:.6f}s budget')
        span = node.publish_setpoints(q, v, a)
        if span > dt:
            raise RuntimeError('Local publication of handover batch exceeded one control period')
        node.expected, last_publish = q.copy(), publication_start
        p, measured_v, _ = node.health(airborne=True)
        settled = (elapsed >= duration and
            np.max(np.linalg.norm(p-goal, axis=1)) < cfg['mission']['endpoint_tolerance_m'] and
            np.max(np.linalg.norm(measured_v, axis=1)) < cfg['mission']['endpoint_speed_mps'])
        final_since = cycle_start if settled and final_since is None else final_since if settled else None
        if final_since is not None and cycle_start-final_since >= cfg['commissioning']['settle_dwell_s']:
            node.result['position_handover'] = {'status': 'PASS', 'first_command_matches_measured_pose': False,
                'first_command_matches_planned_takeoff_reference': True,
                'initial_measured_position_m': measured.tolist(),
                'initial_measured_reference_error_m': initial_measured_error.tolist(),
                'initial_position_m': start.tolist(), 'slot_position_m': goal.tolist(),
                'duration_s': time.monotonic()-begun, 'interface': 'cmd_position',
                'onboard_receipt_verified': False}
            node.get_logger().info('POSITION_HANDOVER_PASS: cf2/cf3/cf4 settled; starting REDCHO ALIGN')
            return
        node.spin_for(max(0., dt-(time.monotonic()-cycle_start)))
    raise RuntimeError('Position handover did not settle; no forward flight started')
