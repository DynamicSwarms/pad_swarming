import time
import unittest

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_testing.actions import ReadyToTest
from pad_management_interfaces.action import PadExecute, PadRightControl
import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup


def megapad_launch_description(container_name, max_requests=None):
    parameters = {
        'pad_resource_manager_plugin': 'megapad::MegaPadResourceManager'
    }
    if max_requests is not None:
        parameters['max_requests'] = max_requests

    megapad = ComposableNode(
        package='pad_management_cpp',
        plugin='PadRightActionServerNode',
        name='megapad',
        parameters=[parameters],
    )
    container = ComposableNodeContainer(
        name=container_name,
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        arguments=['--executor-type', 'multi-threaded'],
        output='screen',
        composable_node_descriptions=[megapad],
    )
    return LaunchDescription([container, ReadyToTest()])


class MegapadTestCase(unittest.TestCase):
    node_name = 'megapad_test'
    executor_threads = 4

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node(cls.node_name)
        cls.executor = rclpy.executors.MultiThreadedExecutor(
            num_threads=cls.executor_threads
        )
        cls.executor.add_node(cls.node)

    @classmethod
    def tearDownClass(cls):
        cls.executor.remove_node(cls.node)
        cls.executor.shutdown(wait_for_threads=True)
        cls.node.destroy_node()
        rclpy.shutdown()

    def spin_until(self, predicate, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return True
            rclpy.spin_once(
                self.node, executor=self.executor, timeout_sec=0.05
            )
        return predicate()

    def create_action_client(self):
        return ActionClient(
            self.node,
            PadRightControl,
            'megapad/pad_right_control',
            callback_group=ReentrantCallbackGroup(),
        )

    def create_execute_server(self, client_id, execute_callback):
        return ActionServer(
            self.node,
            PadExecute,
            f'padflie{client_id}/pad_execute',
            execute_callback=execute_callback,
            callback_group=ReentrantCallbackGroup(),
        )

    def create_releasable_execute_server(self, client_id, release_event):
        def execute(goal_handle):
            if not release_event.wait(timeout=15.0):
                result = PadExecute.Result()
                result.result = PadExecute.Result.RESULT_FAILURE
                result.reason = 'Test timed out waiting for release'
                goal_handle.abort()
                return result

            result = PadExecute.Result()
            result.result = PadExecute.Result.RESULT_ON_PAD
            result.reason = 'Released by test'
            goal_handle.succeed()
            return result

        return self.create_execute_server(client_id, execute)

    def send_request(
        self, client, client_id, feedback_callback, usage_seconds=1.5
    ):
        goal = PadRightControl.Goal()
        goal.name = f'padflie{client_id}'
        goal.action = PadRightControl.Goal.ACTION_TAKEOFF
        goal.max_wait_time = rclpy.duration.Duration(seconds=10.0).to_msg()
        goal.usage_time = rclpy.duration.Duration(
            seconds=usage_seconds
        ).to_msg()
        future = client.send_goal_async(
            goal, feedback_callback=feedback_callback
        )
        self.assertTrue(self.spin_until(future.done))
        self.assertTrue(future.result().accepted)
        return future.result()
