import math
import random
import threading
import time

import rclpy
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from padflies_interfaces.action import Deploy
from padflies_interfaces.msg import PadflieInfo, SendTarget
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class ScalingController(Node):
    def __init__(self):
        super().__init__("crazyflie_scaling_controller")
        self.count = self.declare_parameter("count", 20).value
        self.area = self.declare_parameter("area", 5.0).value
        self.speed = self.declare_parameter("speed", 0.6).value
        self.seed = self.declare_parameter("seed", 42).value
        self.random = random.Random(self.seed)
        self.positions = {}
        self.headings = {
            cf_id: self.random.uniform(-math.pi, math.pi)
            for cf_id in range(self.count)
        }
        self.target_publishers = []
        self.info_subscriptions = []
        self.state_clients = []
        self.transition_clients = []
        self.deploy_clients = []

        for cf_id in range(self.count):
            prefix = f"padflie{cf_id}"
            self.target_publishers.append(
                self.create_publisher(SendTarget, f"{prefix}/send_target", 10)
            )
            self.info_subscriptions.append(
                self.create_subscription(
                    PadflieInfo,
                    f"{prefix}/info",
                    lambda message, identifier=cf_id: self._remember_position(
                        identifier, message
                    ),
                    10,
                )
            )
            self.state_clients.append(self.create_client(GetState, f"{prefix}/get_state"))
            self.transition_clients.append(
                self.create_client(ChangeState, f"{prefix}/change_state")
            )
            self.deploy_clients.append(ActionClient(self, Deploy, f"{prefix}/deploy"))

        self.walk_timer = None

    def start(self):
        self.get_logger().info(f"Preparing {self.count} Crazyflie controllers")
        for cf_id in range(self.count):
            self._activate(cf_id)
        self._deploy_all()
        self.walk_timer = self.create_timer(0.5, self._publish_random_walk)
        self.get_logger().info("All Crazyflies deployed; random walk started")

    def _activate(self, cf_id):
        state_client = self.state_clients[cf_id]
        transition_client = self.transition_clients[cf_id]
        if not state_client.wait_for_service(timeout_sec=30.0):
            raise RuntimeError(f"padflie{cf_id} lifecycle state service unavailable")
        if not transition_client.wait_for_service(timeout_sec=30.0):
            raise RuntimeError(f"padflie{cf_id} lifecycle transition service unavailable")

        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline:
            state = self._call(state_client, GetState.Request(), 2.0).current_state.id
            if state == State.PRIMARY_STATE_ACTIVE:
                return
            if state == State.PRIMARY_STATE_UNCONFIGURED:
                self._change_state(transition_client, Transition.TRANSITION_CONFIGURE)
            elif state == State.PRIMARY_STATE_INACTIVE:
                if self._change_state(transition_client, Transition.TRANSITION_ACTIVATE):
                    return
            time.sleep(0.2)
        raise RuntimeError(f"padflie{cf_id} did not activate")

    def _change_state(self, client, transition_id):
        request = ChangeState.Request()
        request.transition.id = transition_id
        return self._call(client, request, 5.0).success

    def _deploy_all(self):
        for client in self.deploy_clients:
            if not client.wait_for_server(timeout_sec=10.0):
                raise RuntimeError("Deploy action server unavailable")

        accepted = []
        for client in self.deploy_clients:
            goal = Deploy.Goal()
            goal.has_target = False
            accepted.append(self._wait(client.send_goal_async(goal), 5.0))
        if not all(handle.accepted for handle in accepted):
            raise RuntimeError("At least one Deploy goal was rejected")

        results = [handle.get_result_async() for handle in accepted]
        for result_future in results:
            result = self._wait(result_future, 15.0).result
            if result.outcome != Deploy.Result.SUCCESS:
                raise RuntimeError(f"Deploy failed: {result.message}")

    def _publish_random_walk(self):
        for cf_id, publisher in enumerate(self.target_publishers):
            position = self.positions.get(cf_id)
            heading = self.headings[cf_id] + self.random.gauss(0.0, 0.35)
            if position is not None:
                x, y, _ = position
                distance = math.hypot(x, y)
                if distance > self.area:
                    heading = math.atan2(-y, -x) + self.random.uniform(-0.35, 0.35)
            self.headings[cf_id] = heading

            message = SendTarget()
            message.mode_position = False
            message.collision_avoidance = True
            message.velocity.header.frame_id = "world"
            message.velocity.twist.linear.x = self.speed * math.cos(heading)
            message.velocity.twist.linear.y = self.speed * math.sin(heading)
            if position is not None:
                message.velocity.twist.linear.z = max(-0.3, min(0.3, 1.0 - position[2]))
            publisher.publish(message)

    def _remember_position(self, cf_id, message):
        if message.pose_world_valid:
            position = message.pose_world.position
            self.positions[cf_id] = (position.x, position.y, position.z)

    def _call(self, client, request, timeout):
        return self._wait(client.call_async(request), timeout)

    @staticmethod
    def _wait(future, timeout):
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        if not future.done():
            raise TimeoutError("ROS request timed out")
        result = future.result()
        if result is None:
            raise RuntimeError("ROS request failed")
        return result


def main():
    rclpy.init()
    node = ScalingController()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        node.start()
        spin_thread.join()
    except KeyboardInterrupt:
        pass
    except (RuntimeError, TimeoutError) as error:
        if rclpy.ok():
            node.get_logger().error(str(error))
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
