"""Command ownership stays strict when old backends or recorders overlap."""
from types import SimpleNamespace
import pytest
from safe_redcho_single_mission.command_routes import (
    BACKEND, RECORDER, POSITION, endpoints, subscriber_problem, startup_problems,
)


def endpoint(identity):
    ns, name, kind = identity
    return SimpleNamespace(node_namespace=ns, node_name=name, topic_type=kind)


@pytest.mark.parametrize('identities', [[BACKEND], [BACKEND, RECORDER]])
def test_expected_backend_with_optional_recorder(identities):
    assert subscriber_problem(list(map(endpoint, identities))) is None


@pytest.mark.parametrize('identities,reason', [
    ([], 'backend subscribers=0'),
    ([RECORDER], 'backend subscribers=0'),
    ([BACKEND, BACKEND], 'backend subscribers=2'),
    ([BACKEND, RECORDER, RECORDER], 'control recorders=2'),
    ([BACKEND, ('/', 'unexpected_reader', POSITION)], 'unexpected_reader'),
    ([('/other', 'crazyflie_server', POSITION)], 'backend subscribers=0'),
])
def test_bad_endpoint_counts_explain_the_failure(identities, reason):
    assert reason in subscriber_problem(list(map(endpoint, identities)))


def snapshot(publishers=(), subscribers=(), poses=()):
    return {'/poses': {'publishers': endpoints(list(map(endpoint, poses)))},
            '/cf2/cmd_position': {'publishers': endpoints(list(map(endpoint, publishers))),
                                  'subscribers': endpoints(list(map(endpoint, subscribers)))}}


def test_startup_accepts_empty_graph_or_optional_read_only_recorder():
    assert startup_problems(snapshot()) == []
    assert startup_problems(snapshot(subscribers=[RECORDER])) == []


@pytest.mark.parametrize('kwargs,reason', [
    ({'subscribers': [BACKEND]}, 'existing subscriber'),
    ({'publishers': [('/', 'single_mission_coordinator', POSITION)]}, 'existing publisher'),
    ({'poses': [('/', 'motion_capture_tracking', 'motion_capture_tracking_interfaces/msg/NamedPoseArray')]}, '/poses'),
    ({'subscribers': [('/', 'unknown', POSITION)]}, 'unknown'),
])
def test_backend_startup_blocks_existing_stack(kwargs, reason):
    assert reason in '; '.join(startup_problems(snapshot(**kwargs)))
