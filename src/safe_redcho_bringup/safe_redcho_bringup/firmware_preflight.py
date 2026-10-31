"""Read-only, per-parameter verification of the Crazyswarm2 ROS cache.

ROS cache contents are not fresh radio readback or evidence of peer freshness.
Unknown, uninitialized, nonfinite and mismatched values remain failures.
"""
import math
import time

from .onboard_avoidance import firmware_parameters


def expected_parameters(config, names):
    expected = {}
    for name in names:
        expected[f'{name}.params.stabilizer.controller'] = 1
        expected.update({f'{name}.params.colAv.{key}': value
                         for key, value in firmware_parameters(config).items()})
    return expected


def numeric_value(value):
    if value.type == 2:
        return int(value.integer_value)
    if value.type == 3:
        result = float(value.double_value)
        if math.isfinite(result):
            return result
    return None


def verify_parameter_cache(node, config, names, report, *,
                           get_type=None, list_type=None, spin=None, clock=None,
                           request_timeout_s=1.5, max_attempts=3, budget_s=35.):
    if get_type is None or list_type is None:
        from rcl_interfaces.srv import GetParameters, ListParameters
        get_type, list_type = GetParameters, ListParameters
    if spin is None:
        import rclpy
        spin = rclpy.spin_once
    clock = clock or time.monotonic
    expected = expected_parameters(config, names)
    report.update({
        'status': 'CHECKING',
        'scope': 'ROS server parameter cache only; not fresh RF readback',
        'expected_count': len(expected), 'matched_count': 0,
        'parameters': {}, 'missing_declarations': [], 'failures': [],
        'query_strategy': 'sequential_single_parameter_with_bounded_retries_v2',
        'maximum_outstanding_get_requests': 1,
        'get_requests_sent': 0, 'retries_sent': 0,
    })
    clients = []
    active = None

    def wait(future, label, timeout=5.):
        deadline = clock()+timeout
        while not future.done() and clock() < deadline:
            spin(node, timeout_sec=.02)
        if not future.done():
            future.cancel()
            raise RuntimeError(f'{label} timed out')
        response = future.result()
        if response is None:
            raise RuntimeError(f'{label} returned no response')
        return response

    try:
        for service_type, endpoint in (
                (list_type, '/crazyflie_server/list_parameters'),
                (get_type, '/crazyflie_server/get_parameters')):
            client = node.create_client(service_type, endpoint)
            clients.append(client)
            if not client.wait_for_service(timeout_sec=3.):
                raise RuntimeError(f'Parameter service unavailable: {endpoint}')
        list_client, get_client = clients
        request = list_type.Request()
        request.prefixes = [f'{name}.params' for name in names]
        request.depth = 0  # DEPTH_RECURSIVE
        response = wait(list_client.call_async(request), 'Firmware parameter listing')
        declared = set(response.result.names)
        report['declared_required_count'] = len(set(expected) & declared)
        report['missing_declarations'] = sorted(set(expected)-declared)
        # Wait for each reply before sending the next request. The previous
        # implementation burst all 84 requests into finite service queues.
        deadline = clock()+budget_s
        completed = 0
        logger = node.get_logger() if hasattr(node, 'get_logger') else None
        progress_at = clock()
        for name in expected:
            if name not in declared:
                report['parameters'][name] = {
                    'expected': expected[name], 'status': 'MISSING_DECLARATION', 'attempts': 0}
                continue
            result = {'expected': expected[name], 'attempts': 0,
                      'status': 'NOT_QUERIED_BUDGET_EXHAUSTED'}
            for attempt in range(1, max_attempts+1):
                remaining = deadline-clock()
                if remaining <= 0:
                    break
                result['attempts'] = attempt
                request = get_type.Request()
                request.names = [name]
                try:
                    active = get_client.call_async(request)
                    report['get_requests_sent'] += 1
                    if attempt > 1:
                        report['retries_sent'] += 1
                    until = min(deadline, clock()+request_timeout_s)
                    while not active.done() and clock() < until:
                        spin(node, timeout_sec=.02)
                    if not active.done():
                        active.cancel()
                        if hasattr(get_client, 'remove_pending_request'):
                            get_client.remove_pending_request(active)
                        result.update(status='QUERY_TIMEOUT', detail=(
                            'ROS parameter service did not reply within the per-request deadline'))
                        active = None
                        continue
                    response = active.result()
                    result.pop('detail', None)
                    if response is None or len(response.values) != 1:
                        result.update(status='NO_SINGLE_VALUE', detail=(
                            'Declared parameter has no readable value; it may be uninitialized'))
                    else:
                        value = response.values[0]
                        actual = numeric_value(value)
                        result['ros_type'] = int(value.type)
                        result['actual'] = actual
                        if actual is None:
                            result['status'] = 'UNSET_OR_INVALID_TYPE'
                        elif abs(actual-expected[name]) > 1e-5:
                            result['status'] = 'VALUE_MISMATCH'
                        else:
                            result['status'] = 'MATCH'
                            report['matched_count'] += 1
                    # A readable mismatch is a result, not a transport timeout.
                    # Do not retry it until a preferred value happens to appear.
                    break
                except Exception as exc:
                    result.update(status='QUERY_ERROR', detail=str(exc))
                finally:
                    if active is not None and not active.done():
                        active.cancel()
                        if hasattr(get_client, 'remove_pending_request'):
                            get_client.remove_pending_request(active)
                    active = None
            report['parameters'][name] = result
            completed += 1
            if logger and clock()-progress_at >= 1.:
                logger.info(f'Parameter checks: {completed}/{len(expected)} completed; '
                            f'{report["matched_count"]} match')
                progress_at = clock()
        failed = [name for name in expected
                  if report['parameters'][name]['status'] != 'MATCH']
        report['failures'] = failed
        if failed:
            examples = ', '.join(f'{key}: {report["parameters"][key]["status"]}'
                                 for key in failed[:6])
            raise RuntimeError(
                f'PID/BVCA parameter verification failed for {len(failed)}/{len(expected)} values. '
                f'{examples}. Inspect parameter_preflight.json for every actual/expected value. '
                'QUERY_TIMEOUT concerns the ROS service response; it is not a measured firmware mismatch. '
                'Missing declarations and unreadable values require checking backend/firmware setup. '
                'No values were set by this checker.')
        report['status'] = 'PASS_ROS_CACHE'
    except Exception as exc:
        report['status'] = 'FAIL'
        report['error'] = str(exc)
        raise
    finally:
        if active is not None and not active.done():
            active.cancel()
        for client in clients:
            node.destroy_client(client)


def verify_avoidance(self):
    """Replacement FleetMission method; all checks run before arming."""
    from .backend_audit import audit_workspace
    self.avoidance_configuration['routing'] = audit_workspace(self.config)
    record = {}
    self.avoidance_configuration['parameter_check'] = record
    try:
        verify_parameter_cache(self, self.config, self.names, record)
    finally:
        import json
        destination = self.output/'parameter_preflight.json'
        destination.write_text(json.dumps(record, indent=2, allow_nan=False)+'\n')
        self.get_logger().info(f'Parameter diagnostics: {destination}')
    self.avoidance_configuration['ros_parameter_cache_matches'] = True
    self.avoidance_configuration['fresh_onboard_peer_tables_verified'] = False
