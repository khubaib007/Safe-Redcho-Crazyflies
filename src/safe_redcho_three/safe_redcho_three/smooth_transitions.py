"""Normal streamed transitions; independent watchdog remains a fault fallback.
Position packets carry x/y/z/yaw only. Derivatives below are reference diagnostics.
"""
import math
import time
import numpy as np
from .settings import slots, pair_distance, virtual_box
from .commissioning import bridge_sample

VERSION = 'smooth-transitions-1'
DEFAULTS = dict(takeoff_minimum_s=8., descent_minimum_s=10.,
    takeoff_speed_mps=.10, descent_speed_mps=.08, touchdown_speed_mps=.02,
    acceleration_mps2=.12, jerk_mps3=.4, approach_height_m=.08,
    touchdown_minimum_s=8., settle_timeout_s=10., settle_dwell_s=2.,
    settle_position_m=.04, settle_speed_mps=.04,
    contact_position_m=.012, contact_speed_mps=.025, contact_dwell_s=1.,
    contact_timeout_s=6.)


def options(cfg):
    raw = cfg.get('smooth_transitions', {})
    if set(raw)-set(DEFAULTS):
        raise ValueError('Unknown smooth_transitions keys: '+str(set(raw)-set(DEFAULTS)))
    out = dict(DEFAULTS, **raw)
    for k, v in out.items():
        if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or v <= 0:
            raise ValueError('Positive finite smooth_transitions.'+k+' required')
    if out['contact_position_m'] > .015 or out['contact_speed_mps'] > .03 or out['contact_dwell_s'] < 1.:
        raise ValueError('Touchdown evidence must remain tight and sustained')
    if out['takeoff_speed_mps'] > .10 or out['descent_speed_mps'] > .08 or out['touchdown_speed_mps'] > .025:
        raise ValueError('Transition reference speed exceeds commissioning cap')
    if out['approach_height_m'] < .06 or out['contact_position_m'] >= out['approach_height_m']/3:
        raise ValueError('Touchdown approach and contact thresholds overlap')
    return out


def duration(start, goal, minimum, speed, acceleration, jerk):
    d = float(np.max(np.linalg.norm(np.asarray(goal)-start, axis=1)))
    return max(minimum, 1.875*d/speed, math.sqrt(5.773502692*d/acceleration),
               (60*d/jerk)**(1/3))


def corridor(start, goal, cfg):
    """Normal transitions: empty arena, full inflated XY margins; ground Z allowed."""
    start, goal = np.asarray(start, float), np.asarray(goal, float)
    if start.shape != (3, 3) or goal.shape != (3, 3) or not np.isfinite([start, goal]).all():
        raise RuntimeError('Invalid transition geometry')
    if cfg['obstacles']:
        raise RuntimeError('Smooth three-aircraft commissioning requires obstacles=[]')
    lo, hi = virtual_box(cfg)
    if np.any(start[:, :2] <= lo[:2]) or np.any(start[:, :2] >= hi[:2]) or np.any(goal[:, :2] <= lo[:2]) or np.any(goal[:, :2] >= hi[:2]):
        raise RuntimeError('Transition outside inflated XY arena')
    if min(start[:, 2].min(), goal[:, 2].min()) < cfg['arena']['floor_z_m']-.03 or max(start[:, 2].max(), goal[:, 2].max()) >= hi[2]:
        raise RuntimeError('Transition Z outside ground/ceiling envelope')
    for i in range(3):
        for j in range(i):
            r = start[i, :2]-start[j, :2]
            delta = goal[i, :2]-goal[j, :2]-r
            s = np.clip(-r@delta/max(float(delta@delta), 1e-15), 0., 1.)
            if np.linalg.norm(r+s*delta) < pair_distance(cfg):
                raise RuntimeError('Transition lacks full inflated pair clearance')


def stream_segment(node, start, goal, seconds, stage, settle=False):
    cfg, opt = node.profile, options(node.profile)
    corridor(start, goal, cfg)
    node.stage_name = stage
    node.streaming = True
    node.expected = np.asarray(start).copy()
    node.pulse()
    dt = cfg['control']['dt_s']
    begun = last = time.monotonic()
    good_since = None
    maximum_gap = 0.
    while True:
        tick = time.monotonic()
        gap = tick-last
        if gap > cfg['control']['maximum_publish_gap_s']:
            raise RuntimeError(stage+': position publication gap exceeded budget')
        p, measured_v, _ = node.health(airborne=True)
        # Health can itself consume time; check again before publication.
        now = time.monotonic()
        if now-last > cfg['control']['maximum_publish_gap_s']:
            raise RuntimeError(stage+': health checks exceeded publication budget')
        elapsed = now-begun
        q, v, a = bridge_sample(start, goal, elapsed, seconds)
        span = node.publish_setpoints(q, v, a)
        if span > dt:
            raise RuntimeError(stage+': publication batch exceeded one control period')
        maximum_gap = max(maximum_gap, now-last)
        last, node.expected = now, q.copy()
        if elapsed >= seconds:
            good = (np.max(np.linalg.norm(p-goal, axis=1)) < opt['settle_position_m'] and
                    np.max(np.linalg.norm(measured_v, axis=1)) < opt['settle_speed_mps'])
            good_since = now if good and good_since is None else good_since if good else None
            if not settle or (good_since is not None and now-good_since >= opt['settle_dwell_s']):
                node.result.setdefault('smooth_transition_segments', []).append(dict(stage=stage,
                    duration_s=now-begun, planned_duration_s=seconds,
                    maximum_publish_gap_s=maximum_gap, measured_end_position_m=p.tolist(),
                    interface='cmd_position', onboard_receipt_verified=False))
                return q.copy()
            if elapsed > seconds+opt['settle_timeout_s']:
                raise RuntimeError(stage+': measured hover did not settle')
        node.spin_for(max(0., dt-(time.monotonic()-tick)))


def takeoff(node):
    from crazyflie_interfaces.srv import Arm
    opt = options(node.profile)
    node.stage_name = 'GROUND'
    node.health(ground=True)
    start, _, _ = node.health(ground=True)
    vertical_goal = start.copy()
    vertical_goal[:, 2] = slots(node.profile)[:, 2]
    corridor(start, vertical_goal, node.profile)
    corridor(vertical_goal, slots(node.profile), node.profile)
    node.arm_attempted = True
    requests = {}
    for n in node.names:
        req = Arm.Request(); req.arm = True; requests[n] = req
    node.call_many('arm', requests)
    # From this point uncertainty must retain landing/watchdog recovery.
    node.flight_active, node.airborne_possible = True, True
    node.streaming, node.expected = True, start.copy()
    node.pulse()
    node.publish_setpoints(start, np.zeros_like(start), np.zeros_like(start))
    seconds = duration(start, vertical_goal,
        max(opt['takeoff_minimum_s'], node.profile['mission']['takeoff_duration_s']),
        opt['takeoff_speed_mps'], opt['acceleration_mps2'], opt['jerk_mps3'])
    stream_segment(node, start, vertical_goal, seconds, 'SMOOTH_TAKEOFF', settle=True)
    # Small horizontal correction only after stable hover; no high-level mode switch.
    seconds = duration(vertical_goal, slots(node.profile),
        node.profile['commissioning']['handover_duration_s'], .04,
        opt['acceleration_mps2'], opt['jerk_mps3'])
    stream_segment(node, vertical_goal, slots(node.profile), seconds, 'SMOOTH_SLOT_ALIGN', settle=True)
    node.result['position_handover'] = dict(status='PASS', interface='cmd_position',
        mode='continuous streamed takeoff and slot alignment',
        first_command_matches_measured_pose=True, onboard_receipt_verified=False)
    node._smooth_takeoff_complete = True


def contact_gate(node, target):
    """Keep streaming floor target; disarm eligibility requires current stable contact."""
    opt = options(node.profile)
    node.stage_name = 'SMOOTH_CONTACT_VERIFY'
    begun = last = time.monotonic()
    good_since = None
    while time.monotonic()-begun < opt['contact_timeout_s']:
        tick = time.monotonic()
        p, v, _ = node.health(airborne=True)
        if time.monotonic()-last > node.profile['control']['maximum_publish_gap_s']:
            raise RuntimeError('Contact hold publication gap exceeded budget')
        span = node.publish_setpoints(target, np.zeros_like(target), np.zeros_like(target))
        if span > node.profile['control']['dt_s']:
            raise RuntimeError('Contact publication batch exceeded control period')
        last = time.monotonic()
        good = (np.max(np.abs(p[:, 2]-target[:, 2])) <= opt['contact_position_m'] and
                np.max(np.linalg.norm(v, axis=1)) <= opt['contact_speed_mps'])
        good_since = tick if good and good_since is None else good_since if good else None
        if good_since is not None and tick-good_since >= opt['contact_dwell_s']:
            return p, v
        node.spin_for(max(0., node.profile['control']['dt_s']-(time.monotonic()-tick)))
    raise RuntimeError('Fresh sustained ground contact not established; no smooth-path disarm')


def land(node):
    from crazyflie_interfaces.srv import Arm
    opt = options(node.profile)
    p, velocity, _ = node.health(airborne=True)
    if np.max(np.linalg.norm(velocity, axis=1)) >= opt['settle_speed_mps']:
        raise RuntimeError('Normal smooth landing requires settled final hover')
    # Continue from the last transmitted reference, avoiding an initial packet jump.
    start = np.asarray(node.expected, float).copy()
    ground = start.copy()
    ground[:, 2] = [node.config['_starts'][n][2] for n in node.names]
    approach = ground.copy()
    approach[:, 2] += opt['approach_height_m']
    if np.any(start[:, 2] <= approach[:, 2]):
        raise RuntimeError('Normal landing must start above approach height')
    seconds = duration(start, approach,
        max(opt['descent_minimum_s'], node.profile['mission']['landing_duration_s']),
        opt['descent_speed_mps'], opt['acceleration_mps2'], opt['jerk_mps3'])
    stream_segment(node, start, approach, seconds, 'SMOOTH_DESCENT')
    seconds = duration(approach, ground, opt['touchdown_minimum_s'],
        opt['touchdown_speed_mps'], opt['acceleration_mps2'], opt['jerk_mps3'])
    stream_segment(node, approach, ground, seconds, 'SMOOTH_TOUCHDOWN')
    p, v = contact_gate(node, ground)
    requests = {}
    for n in node.names:
        req = Arm.Request(); req.arm = False; requests[n] = req
    node.call_many('arm', requests)
    node.streaming = False
    node.flight_active = False
    node.pulse()
    node.stage_name = 'LANDED'
    # No Land or Notify service on the successful path: no airborne mode handover.
    return dict(mode=VERSION, touchdown_verified=list(node.names), touchdown_not_verified=[],
        disarm_requests_acknowledged=list(node.names),
        touchdown_evidence={n: dict(verified=True,
            ground_height_error_m=float(abs(p[i, 2]-ground[i, 2])),
            measured_speed_mps=float(np.linalg.norm(v[i])),
            sustained_s=opt['contact_dwell_s']) for i, n in enumerate(node.names)},
        onboard_receipt_verified=False)
