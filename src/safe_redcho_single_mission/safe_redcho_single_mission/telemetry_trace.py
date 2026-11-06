"""Optional observations of the existing arithmetic; no control decisions here."""
import queue
import threading
import time
import numpy as np


def command_subscribers_valid(node, names):
    """One actual backend plus at most one known read-only control recorder."""
    expected_type = 'crazyflie_interfaces/msg/Position'
    for name in names:
        endpoints = node.get_subscriptions_info_by_topic(f'/{name}/cmd_position')
        identities = [(e.node_namespace, e.node_name, e.topic_type) for e in endpoints]
        backend = [x for x in identities if x == ('/', 'crazyflie_server', expected_type)]
        recorders = [x for x in identities if x == ('/', 'single_redcho_telemetry_control', expected_type)]
        if len(backend) != 1 or len(recorders) > 1 or len(backend)+len(recorders) != len(identities):
            return False
    return True


class QPTraceError(RuntimeError):
    def __init__(self, message, diagnostics):
        super().__init__(message)
        self.diagnostics = diagnostics


def reference_trace(cfg, index, frame, before_phase, before_state, velocity_input, position_input, ref):
    dt = cfg['control']['dt_s']
    duration = cfg['mission']['nominal_forward_duration_s']
    smooth = lambda s: s**3*(10-15*s+6*s*s)
    unscaled_phase = min(1., before_phase+dt/duration) if frame['stage'] == 'FORWARD' else before_phase
    unscaled = [cfg['mission']['distance_m']*(smooth(unscaled_phase)-smooth(before_phase))/dt, 0., 0.]
    omega = cfg['redcho']['svf_omega']
    raw = np.asarray(ref['raw_velocity'])
    snap = -omega**3*(ref['svf_after'][:, 1]-raw)-3*omega**2*ref['svf_after'][:, 2]-3*omega*ref['svf_after'][:, 3]
    vector = lambda a: dict(zip(('x', 'y', 'z'), np.asarray(a).tolist()))
    return {
        'governor': {
            'input_voltage_v': frame['voltage'][index],
            'input_stage': frame['stage'],
            'mission_distance_m': cfg['mission']['distance_m'],
            'nominal_duration_s': duration,
            'minimum_speed_scale': cfg['battery']['minimum_speed_scale'],
            'land_voltage_v': cfg['battery']['land_voltage_v'],
            'full_speed_voltage_v': cfg['battery']['full_speed_voltage_v'],
            'input_unscaled_velocity_mps': vector(unscaled),
            'phase_before': before_phase, 'phase_after': ref['phase_after'],
            'phase_increment': ref['phase_after']-before_phase,
            'speed_scale': ref['battery_scale'],
            'output_velocity_mps': vector(raw),
            'note': 'Unscaled input is evaluated at the same local phase; not a separate unslowed mission.',
        },
        'svf': {
            'input_velocity_mps': vector(raw),
            'state_before': before_state.tolist(), 'state_after': ref['svf_after'].tolist(),
            'output_displacement_m': vector(ref['svf_after'][:, 0]),
            'output_velocity_mps': vector(velocity_input[:, 0]),
            'output_acceleration_mps2': vector(velocity_input[:, 1]),
            'output_jerk_mps3': vector(velocity_input[:, 2]),
            'poststep_snap_mps4': vector(snap),
            'state_column_order': 'integral_velocity,velocity,acceleration,jerk',
        },
    }


def bank_trace(reference, output, before, after, peers, prefix):
    vector = lambda a: dict(zip(('x', 'y', 'z'), np.asarray(a).tolist()))
    labels = ('position_m', 'velocity_mps', 'acceleration_mps2')
    return {
        'input': {label: vector(reference[:, k]) for k, label in enumerate(labels)},
        'output': {label: vector(output[:, k]) for k, label in enumerate(labels)},
        'internal_state_before': before.tolist(), 'internal_state_after': after.tolist(),
        'neighbor_zeroth_order': peers,
        'position_semantics': 'absolute world center and height proposal' if prefix == 'position' else '',
    }


def barrier_rows(index, q, v, previous, value, A, b, labels, cfg):
    from .settings import pair_distance, virtual_box
    s, k1 = cfg['safety'], cfg['safety']['hocbf_k1']
    lo, hi = virtual_box(cfg)
    data = {}
    for row, label in enumerate(labels):
        key=label[5:] if label.startswith('next_') else label
        if key.startswith('pair_'):
            j = cfg['names'].index(key[5:])
            r, rv = q[index, :2]-q[j, :2], v[index, :2]-v[j, :2]
            h, hd = float(r@r-pair_distance(cfg)**2), float(2*r@rv)
            unit = 'm^2'
        elif key.startswith('upper_') or key.startswith('lower_'):
            axis = int(key.split('_')[1]); upper = key.startswith('upper_')
            h = float(hi[axis]-q[index, axis] if upper else q[index, axis]-lo[axis])
            hd = float(-v[index, axis] if upper else v[index, axis]); unit = 'm'
        else:
            obstacle = cfg['obstacles'][int(key.split('_')[1])]
            r, rv = q[index, :2]-obstacle['center_xy'], v[index, :2]
            from .obstacles import obstacle_radius
            radius = obstacle_radius(obstacle, cfg)
            h, hd = float(r@r-radius**2), float(2*r@rv); unit = 'm^2'
        residual = float(b[row]-A[row]@value)
        data[label] = {'A': A[row].tolist(), 'b': float(b[row]), 'residual_b_minus_Au': residual,
                       'h': h, 'h_dot': hd, 'psi1': hd+k1*h, 'h_unit': unit,
                       'binding': residual <= 10*cfg['control']['qp_tolerance'],
                       'row_family':'next_step_psi1' if label.startswith('next_') else 'continuous_hocbf'}
    return data


def qp_trace(index, nominal, q, v, previous, value, A, b, labels, lower, upper,
             clipped, iterations, elapsed, violation, result, cfg):
    rows = barrier_rows(index, q, v, previous, value, A, b, labels, cfg)
    binding = any(row['binding'] for row in rows.values())
    correction = float(np.linalg.norm(value-nominal))
    feasible = bool(np.isfinite(value).all() and violation <= cfg['control']['qp_tolerance'])
    return {
        'nominal_acceleration_mps2': nominal.tolist(), 'filtered_acceleration_mps2': value.tolist(),
        'reference_position_m': q.tolist(), 'reference_velocity_mps': v.tolist(),
        'previous_acceleration_mps2': previous.tolist(),
        'lower_acceleration_bound_mps2': lower.tolist(), 'upper_acceleration_bound_mps2': upper.tolist(),
        'lower_bound_residual_mps2': (value-lower).tolist(), 'upper_bound_residual_mps2': (upper-value).tolist(),
        'bounded_weighted_minimizer_mps2': clipped.tolist(),
        'rows': rows, 'intervention_mps2': (value-nominal).tolist(), 'intervention_norm_mps2': correction,
        'inequality_solver_invoked': result is not None, 'hocbf_rows_binding': binding,
        'nominal_hocbf_violation_mps2': float(max(0., np.max(A@nominal-b))),
        'bounded_free_hocbf_violation_mps2': float(max(0., np.max(A@clipped-b))),
        'solver_success': bool(result.success) if result is not None else True,
        'solver_status': int(result.status) if result is not None else 0,
        'solver_message': str(result.message) if result is not None else 'Feasible bounded weighted objective minimizer; SLSQP not required',
        'solver_iterations': iterations, 'solver_wall_s': elapsed, 'accepted_feasible': feasible,
        'max_violation_mps2': violation,
        'note': 'Correction can arise from smoothing, box/slew bounds, or HOCBF rows. It is not alone proof of obstacle intervention.',
    }


class DiagnosticEmitter:
    """Best-effort diagnostics on a bounded worker queue, after command publish."""
    def __init__(self, node, name):
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        from std_msgs.msg import String
        self.String, self.node = String, node
        qos = QoSProfile(depth=8, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.publisher = node.create_publisher(String, f'/safe_redcho_single_mission/{name}/algorithm_telemetry', qos)
        self.queue, self.dropped, self.errors, self.closed = queue.Queue(maxsize=16), 0, 0, False
        self.worker = threading.Thread(target=self._run, name=f'{name}_diagnostics', daemon=True)
        self.worker.start()

    def submit(self, packet):
        if self.closed: return
        packet['emitter_dropped_before'] = self.dropped
        packet['emitter_errors_before'] = self.errors
        try: self.queue.put_nowait(packet)
        except queue.Full: self.dropped += 1

    def _run(self):
        from .protocol import encode
        while True:
            packet = self.queue.get()
            try:
                if packet is None: return
                self.publisher.publish(self.String(data=encode(packet)))
            except Exception:
                self.errors += 1
            finally: self.queue.task_done()

    def close(self):
        self.closed = True
        try: self.queue.put(None, timeout=.3)
        except queue.Full: return
        self.worker.join(timeout=1.)
