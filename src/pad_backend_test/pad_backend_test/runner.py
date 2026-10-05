"""Run one experiment. Wall-clock deadlines remain valid if simulation stalls."""
import argparse
import json
from pathlib import Path
import signal
import time
from datetime import datetime, timezone

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.action import ActionClient
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import qos_profile_sensor_data
from action_msgs.msg import GoalStatus
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import GetState, ChangeState
from padflies_interfaces.action import Deploy, Return
from padflies_interfaces.msg import PadflieInfo, SendTarget
from rosidl_runtime_py.convert import message_to_ordereddict
from ament_index_python.packages import get_package_share_directory
from .sequence import validate


class Experiment(Node):
    def __init__(self, args, output):
        super().__init__('pad_backend_velocity_test', parameter_overrides=[
            Parameter('use_sim_time', value=args.use_sim_time)])
        self.args, self.output = args, output
        self.started = time.monotonic()
        self.phase = 'connect'
        self.sequence_origin = None
        self.latest = None
        self.last_info = 0.0
        self.samples = 0
        self.stop = False
        self.cleaning = False
        prefix = f'/padflie{args.id}'
        self.pub = self.create_publisher(SendTarget, prefix + '/send_target', 10)
        self.sub = self.create_subscription(PadflieInfo, prefix + '/info', self.info, qos_profile_sensor_data)
        self.state_client = self.create_client(GetState, prefix + '/get_state')
        self.change_client = self.create_client(ChangeState, prefix + '/change_state')
        self.deploy = ActionClient(self, Deploy, prefix + '/deploy')
        self.return_client = ActionClient(self, Return, prefix + '/return')

    def record(self, kind, **values):
        self.output.write(json.dumps(dict(kind=kind, elapsed=time.monotonic()-self.started,
                                         ros_time_ns=self.get_clock().now().nanoseconds,
                                         sequence_elapsed=None if self.sequence_origin is None else self.sequence_time()-self.sequence_origin,
                                         phase=self.phase, **values)) + '\n')
        self.output.flush()

    def info(self, msg):
        self.latest, self.last_info = msg, time.monotonic()
        self.samples += 1
        self.record('info', message=message_to_ordereddict(msg))

    def sequence_time(self):
        return self.get_clock().now().nanoseconds / 1e9 if self.args.use_sim_time else time.monotonic()

    def spin(self):
        rclpy.spin_once(self, timeout_sec=0.02)
        if self.stop and not self.cleaning:
            raise InterruptedError('Interrupted; returning aircraft')

    def until(self, predicate, timeout, label):
        deadline = time.monotonic() + timeout
        while not predicate():
            if time.monotonic() >= deadline:
                raise TimeoutError(label)
            self.spin()

    def result(self, future, timeout):
        self.until(future.done, timeout, 'ROS response timed out')
        return future.result()

    def service(self, client, request):
        self.until(client.service_is_ready, self.args.timeout, 'Service unavailable')
        return self.result(client.call_async(request), self.args.timeout)

    def state(self):
        return self.service(self.state_client, GetState.Request()).current_state.id

    def transition(self, transition):
        request = ChangeState.Request()
        request.transition.id = transition
        if not self.service(self.change_client, request).success:
            raise RuntimeError(f'Lifecycle transition {transition} failed')

    def action(self, client, goal):
        self.until(client.server_is_ready, self.args.timeout, 'Action unavailable')
        handle = self.result(client.send_goal_async(goal), self.args.timeout)
        if not handle.accepted:
            raise RuntimeError('Action rejected')
        try:
            result = self.result(handle.get_result_async(), self.args.timeout)
        except BaseException:
            # Return supersedes deploy; request cancellation before starting cleanup.
            handle.cancel_goal_async()
            raise
        self.record('action_result', status=result.status, message=message_to_ordereddict(result.result))
        if result.status != GoalStatus.STATUS_SUCCEEDED or result.result.outcome != 0:
            raise RuntimeError(f'Action failed: {result.result.message}')

    def velocity(self, values, name):
        msg = SendTarget()
        msg.mode_position = False
        msg.collision_avoidance = True
        msg.use_yaw_velocity = True
        msg.velocity.header.frame_id = 'world'
        msg.velocity.header.stamp = self.get_clock().now().to_msg()
        msg.velocity.twist.linear.x, msg.velocity.twist.linear.y, msg.velocity.twist.linear.z = map(float, values[:3])
        msg.velocity.twist.angular.z = float(values[3])
        msg.info = name
        self.pub.publish(msg)
        self.record('command', message=message_to_ordereddict(msg))

    def run(self, sequence):
        owned = False
        try:
            if getattr(self.args, 'use_sim_time', False):
                self.until(lambda: self.get_clock().now().nanoseconds > 0,
                           self.args.timeout, 'No simulation clock received')
            # Padflie configures itself when its underlying cf appears.
            self.until(lambda: self.state() == State.PRIMARY_STATE_INACTIVE,
                       self.args.timeout, 'Padflie did not become inactive')
            owned = True  # Activation may succeed even if its response is lost.
            self.transition(Transition.TRANSITION_ACTIVATE)
            self.until(lambda: self.latest is not None and self.latest.pose_world_valid,
                       self.args.timeout, 'No valid world pose')
            self.phase = 'deploy'
            self.action(self.deploy, Deploy.Goal())
            self.until(lambda: self.pub.get_subscription_count() > 0,
                       self.args.timeout, 'No velocity subscriber')
            self.sequence_origin = self.sequence_time()
            for step in sequence:
                self.phase = step['name']
                start = self.sequence_time()
                end = start + step['duration']
                next_command = start
                previous = start
                progressed = time.monotonic()
                before = self.samples
                while True:
                    self.spin()
                    now = self.sequence_time()
                    if now < previous:
                        raise RuntimeError('Sequence clock moved backwards')
                    if now > previous:
                        progressed = time.monotonic()
                    elif time.monotonic() - progressed > self.args.clock_timeout:
                        raise RuntimeError('Simulation clock stalled')
                    previous = now
                    if now >= end:
                        break
                    if time.monotonic() - self.last_info > self.args.info_timeout:
                        raise RuntimeError('Info stream stale')
                    if not self.latest.pose_world_valid or self.latest.battery == PadflieInfo.BATTERY_STATE_CRITICAL:
                        raise RuntimeError('Invalid pose or critical battery')
                    if now >= next_command:
                        self.velocity(step['velocity'], step['name'])
                        # Skip missed slots rather than burst stale commands.
                        next_command += (int((now-next_command)*self.args.rate)+1) / self.args.rate
                if self.samples == before:
                    raise RuntimeError('No info samples during step')
        finally:
            if owned:
                self.cleaning = True
                self.phase = 'return'
                self.velocity([0, 0, 0, 0], 'cleanup')
                try:
                    self.action(self.return_client, Return.Goal())
                finally:
                    self.phase = 'deactivate'
                    if self.state() == State.PRIMARY_STATE_ACTIVE:
                        # Normal deactivation also invokes the commander's return path.
                        self.transition(Transition.TRANSITION_DEACTIVATE)
                    if self.state() != State.PRIMARY_STATE_INACTIVE:
                        raise RuntimeError('Padflie did not deactivate')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--backend', choices=['simulation', 'sitl', 'hardware'], required=True)
    parser.add_argument('--id', type=lambda s: int(s, 0), required=True)
    parser.add_argument('--sequence', default=str(Path(get_package_share_directory('pad_backend_test')) / 'config/sequence.json'))
    parser.add_argument('--output', default='velocity-results')
    parser.add_argument('--use-sim-time', action='store_true', help='Use /clock for step durations and command rate')
    parser.add_argument('--clock-timeout', type=float, default=5.0, help='Maximum stalled-clock wall seconds')
    parser.add_argument('--rate', type=float, default=20.0)
    parser.add_argument('--timeout', type=float, default=90.0)
    parser.add_argument('--info-timeout', type=float, default=1.0)
    args = parser.parse_args()
    import math
    if args.id < 0 or any(not math.isfinite(v) or v <= 0 for v in (args.rate, args.timeout, args.info_timeout, args.clock_timeout)):
        parser.error('ID must be nonnegative; rate and timeouts must be finite and positive')
    if args.use_sim_time and args.backend != 'simulation':
        parser.error('--use-sim-time is supported for the simulation backend only')
    sequence = validate(json.loads(Path(args.sequence).read_text()))
    folder = Path(args.output).expanduser()
    folder.mkdir(parents=True, exist_ok=True)
    path = folder / f'{args.backend}-cf{args.id}-{datetime.now(timezone.utc):%Y%m%dT%H%M%S.%fZ}.jsonl'
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = None
    previous = {}
    try:
        with path.open('x') as output:
            node = Experiment(args, output)
            for sig in (signal.SIGINT, signal.SIGTERM):
                previous[sig] = signal.signal(sig, lambda *_: setattr(node, 'stop', True))
            node.record('metadata', backend=args.backend, id=args.id, sequence=sequence, rate=args.rate, use_sim_time=args.use_sim_time)
            try:
                node.run(sequence)
            except BaseException as error:
                node.record('result', success=False, error=str(error))
                raise
            else:
                node.record('result', success=True, info_samples=node.samples)
    finally:
        for sig, handler in previous.items():
            signal.signal(sig, handler)
        if node:
            node.destroy_node()
        rclpy.shutdown()
        print(f'Recording: {path.resolve()}')
