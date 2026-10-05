"""Exercise failure cleanup without ROS nodes or aircraft."""
from types import SimpleNamespace
import pytest
from lifecycle_msgs.msg import State, Transition
from pad_backend_test.runner import Experiment


class Fake:
    run = Experiment.run

    def __init__(self, fail_return=False):
        self.args = SimpleNamespace(timeout=1)
        self.latest = SimpleNamespace(pose_world_valid=True)
        self.deploy, self.return_client = 'deploy', 'return'
        self.active = False
        self.events = []
        self.fail_return = fail_return

    def until(self, predicate, *args):
        assert predicate()

    def state(self):
        return State.PRIMARY_STATE_ACTIVE if self.active else State.PRIMARY_STATE_INACTIVE

    def transition(self, value):
        self.events.append(value)
        self.active = value == Transition.TRANSITION_ACTIVATE

    def action(self, client, goal):
        self.events.append(client)
        if client == 'deploy':
            raise RuntimeError('deploy failed')
        if self.fail_return:
            raise RuntimeError('return failed')

    def velocity(self, values, name):
        self.events.append(('velocity', values))


@pytest.mark.parametrize('fail_return', [False, True])
def test_deactivate_even_when_actions_fail(fail_return):
    fake = Fake(fail_return)
    with pytest.raises(RuntimeError):
        fake.run([])
    assert fake.events == [Transition.TRANSITION_ACTIVATE, 'deploy',
                           ('velocity', [0, 0, 0, 0]), 'return', Transition.TRANSITION_DEACTIVATE]
    assert not fake.active
