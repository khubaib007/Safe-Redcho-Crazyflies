"""No-radio numerical validation of the actual local-controller implementation.

The tracking plant is explicitly a damped double integrator, not Crazyflie
firmware, Gazebo, or a validation of the hardware tracking-tube assumption.
"""
import argparse
import json
from pathlib import Path
import time
import numpy as np
from .settings import load, slots, pair_distance, virtual_box
from .controller import LocalController
from .cbf_qp import joint_pair_margin, validate_batch


def run(cfg, output=None, disturbance=True):
    dt = cfg['control']['dt_s']
    agents = [LocalController(cfg, i) for i in range(6)]
    q, v, previous = slots(cfg), np.zeros((6, 3)), np.zeros((6, 3))
    p, measured_v = q.copy(), v.copy()
    history, all_done_since, started = [], None, time.perf_counter()
    minimum_pair, minimum_real, maximum_tracking, maximum_violation = np.inf, np.inf, 0., 0.
    minimum_margin = np.inf
    for cycle in range(1, int(120/dt)+1):
        t = (cycle-1)*dt
        stage = 'ALIGN' if t < cfg['mission']['observer_warmup_s'] else 'FORWARD'
        voltage = np.array([4., 3.95, 3.82, 3.72, 3.65, 3.60])
        frame = {'seq': cycle, 'stage': stage, 'q': q.tolist(), 'v': v.tolist(),
                 'previous': previous.tolist(), 'voltage': voltage.tolist()}
        outgoing = {a.name: a.prepare(frame) for a in agents}
        results = [a.finish({n: outgoing[n] for n in a.neighbors}) for a in agents]
        u = np.array([r['u'] for r in results])
        minimum_margin = min(minimum_margin, joint_pair_margin(q, v, u, cfg))
        maximum_violation = max(maximum_violation, max(r['qp_violation'] for r in results))
        q, v = validate_batch(q, v, previous, u, cfg)
        previous = u
        forcing = np.zeros((6, 3))
        if disturbance:
            forcing[:, 0] = .025*np.sin(.7*t+np.arange(6))
            forcing[:, 1] = .02*np.sin(.5*t+np.arange(6))
            forcing[:, 2] = .015*np.sin(.9*t+np.arange(6))
        actual_acceleration = 9*(q-p)+6*(v-measured_v)+forcing
        p = p+dt*measured_v+0.5*dt*dt*actual_acceleration
        measured_v += dt*actual_acceleration
        pair = min(np.linalg.norm(q[i, :2]-q[j, :2]) for i in range(6) for j in range(i))
        real_pair = min(np.linalg.norm(p[i, :2]-p[j, :2]) for i in range(6) for j in range(i))
        tracking = max(np.linalg.norm(q-p, axis=1))
        minimum_pair, minimum_real = min(minimum_pair, pair), min(minimum_real, real_pair)
        maximum_tracking = max(maximum_tracking, tracking)
        lo, hi = virtual_box(cfg)
        if np.any(q < lo) or np.any(q > hi) or pair < pair_distance(cfg)-1e-5:
            raise RuntimeError(f'Offline virtual safety envelope violated at t={t}')
        targets = np.array([r['target'] for r in results])
        history.append((t, q.copy(), p.copy(), v.copy(), targets.copy(),
                        np.array([r['velocity_estimate'] for r in results]),
                        np.array([r['position_estimate'] for r in results]), u.copy()))
        end = slots(cfg)+[cfg['mission']['distance_m'], 0, 0]
        if all(r['local_phase'] >= 1 for r in results) and np.max(np.linalg.norm(q-end, axis=1)) < .01 and np.max(np.linalg.norm(v, axis=1)) < .01:
            all_done_since = t if all_done_since is None else all_done_since
            if t-all_done_since >= 3: break
        else:
            all_done_since = None
    else:
        raise RuntimeError('Offline mission did not finish within 120 logical seconds')
    endpoint_errors = np.linalg.norm(p-end, axis=1)
    result = {'status': 'PASS', 'scope': 'Numerical reference governor + synthetic tracking plant; no ROS/radio/hardware',
              'duration_s': float(t), 'wall_s': time.perf_counter()-started,
              'minimum_virtual_xy_separation_m': float(minimum_pair),
              'required_virtual_xy_separation_m': pair_distance(cfg),
              'minimum_synthetic_measured_xy_separation_m': float(minimum_real),
              'maximum_synthetic_tracking_error_m': float(maximum_tracking),
              'maximum_qp_violation': float(maximum_violation), 'minimum_joint_hocbf_margin': float(minimum_margin),
              'final_synthetic_endpoint_errors_m': endpoint_errors.tolist(),
              'final_virtual_formation_error_m': float(np.max(np.linalg.norm(q-end, axis=1))),
              'final_virtual_height_spread_m': float(np.ptp(q[:, 2]))}
    if output:
        path = Path(output)
        path.mkdir(parents=True, exist_ok=True)
        (path/'offline_report.json').write_text(json.dumps(result, indent=2)+'\n')
        keys = ['time', 'reference_position', 'synthetic_position', 'reference_velocity', 'formation_target',
                'velocity_redcho', 'pose_redcho', 'safe_acceleration']
        arrays = {k: np.array([row[i] for row in history]) for i, k in enumerate(keys)}
        np.savez_compressed(path/'offline_trace.npz', **arrays)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    parser.add_argument('--output', default='offline_redcho_results')
    args = parser.parse_args()
    print(json.dumps(run(load(args.config), args.output), indent=2))


if __name__ == '__main__': main()
