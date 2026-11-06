"""Read-only command ownership checks; never stop another process."""
from datetime import datetime, timezone
import json
from pathlib import Path
import time
import uuid

POSITION = 'crazyflie_interfaces/msg/Position'
BACKEND = ('/', 'crazyflie_server', POSITION)
RECORDER = ('/', 'single_redcho_telemetry_control', POSITION)


def endpoints(items):
    return [dict(namespace=e.node_namespace, name=e.node_name, type=e.topic_type)
            for e in items]


def subscriber_problem(items):
    identities = [(e.node_namespace, e.node_name, e.topic_type) for e in items]
    backends, recorders = identities.count(BACKEND), identities.count(RECORDER)
    unknown = [x for x in identities if x not in (BACKEND, RECORDER)]
    if backends == 1 and recorders <= 1 and not unknown:
        return None
    return (f'backend subscribers={backends} (need 1), control recorders={recorders} '
            f'(allow 0 or 1), unexpected={unknown}; '
            'with aircraft grounded/disarmed, close obsolete backend/observer launches')


def startup_snapshot(node, names):
    snapshot = {'/poses': {'publishers': endpoints(node.get_publishers_info_by_topic('/poses'))}}
    for name in names:
        topic = f'/{name}/cmd_position'
        snapshot[topic] = dict(publishers=endpoints(node.get_publishers_info_by_topic(topic)),
                              subscribers=endpoints(node.get_subscriptions_info_by_topic(topic)))
    return snapshot


def startup_problems(snapshot):
    problems = []
    for topic, routes in snapshot.items():
        if routes['publishers']:
            problems.append(f'{topic}: existing publisher(s) {routes["publishers"]}')
        for e in routes.get('subscribers', []):
            if (e['namespace'], e['name'], e['type']) != RECORDER:
                problems.append(f'{topic}: existing subscriber {e}')
    return problems


def require_clear_backend_start(cfg, discovery_s=3.):
    """Refuse an overlapping backend/mocap launch before either node starts."""
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    context = Context()
    rclpy.init(context=context)
    node = None
    executor = None
    try:
        node = Node('single_backend_startup_probe_' + uuid.uuid4().hex[:8], context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        end = time.monotonic() + discovery_s
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=min(.1, max(0., end-time.monotonic())))
        snapshot = startup_snapshot(node, cfg['names'])
        problems = startup_problems(snapshot)
        report = dict(status='REFUSED' if problems else 'CLEAR', endpoints=snapshot, problems=problems)
        directory = Path(cfg['workspace']).expanduser()/'results/single_backend_startup'
        directory.mkdir(parents=True, exist_ok=True)
        path = directory/(datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')+'_'+uuid.uuid4().hex[:6]+'.json')
        path.write_text(json.dumps(report, indent=2))
        if problems:
            raise RuntimeError('Backend not started: '+ '; '.join(problems)+
                               '. Close the old stack/standalone mocap while grounded and disarmed. Report: '+str(path))
        return report
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        context.try_shutdown()
