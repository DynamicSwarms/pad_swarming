import threading
import time
import unittest

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
import launch_testing
from pad_management_interfaces.action import PadExecute, PadRightControl
from pad_management_interfaces.msg import SiteInfo
import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from smart_pad_interfaces.srv import Lock


PAD_COUNT = 5


def generate_test_description():
    smart_pads = [
        ComposableNode(
            package='pad_management_cpp',
            plugin='PadRightActionServerNode',
            name=f'smart_pad_{pad_id}',
            parameters=[{
                'pad_resource_manager_plugin':
                    'smart_pad::SmartPadResourceManager',
                'id': pad_id,
            }],
        )
        for pad_id in range(PAD_COUNT)
    ]
    transforms = [
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--x', f'{1.0 + pad_id * 0.4}',
                '--y', '1.2',
                '--z', '0.05',
                '--yaw', '0',
                '--pitch', '0',
                '--roll', '0',
                '--frame-id', 'world',
                '--child-frame-id', f'smart_pad_{pad_id}',
            ],
        )
        for pad_id in range(PAD_COUNT)
    ]
    container = ComposableNodeContainer(
        name='smart_pad_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        arguments=['--executor-type', 'multi-threaded'],
        output='screen',
        composable_node_descriptions=smart_pads,
    )
    return LaunchDescription([
        container,
        *transforms,
        launch_testing.actions.ReadyToTest(),
    ])


class TestSmartPad(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_smart_pad')
        cls.executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
        cls.executor.add_node(cls.node)
        cls.site_info_names = set()
        cls.subscription = cls.node.create_subscription(
            SiteInfo,
            'pad_management/site_info',
            lambda message: cls.site_info_names.add(message.name),
            rclpy.qos.QoSProfile(
                depth=10,
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )

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

    def create_pad_right_client(self):
        return ActionClient(
            self.node,
            PadRightControl,
            'smart_pad_0/pad_right_control',
            callback_group=ReentrantCallbackGroup(),
        )

    def create_goal(self):
        goal = PadRightControl.Goal()
        goal.name = 'padflie1'
        goal.action = PadRightControl.Goal.ACTION_TAKEOFF
        goal.max_wait_time = rclpy.duration.Duration(seconds=5.0).to_msg()
        goal.usage_time = rclpy.duration.Duration(seconds=1.0).to_msg()
        return goal

    def test_all_pads_publish_discovery_info(self):
        self.assertTrue(
            self.spin_until(lambda: len(self.site_info_names) >= PAD_COUNT),
            'Not all smart pads published SiteInfo',
        )

    def test_neighbor_lock_service_is_available(self):
        client = self.node.create_client(Lock, 'smart_pad_0/lock')
        try:
            self.assertTrue(client.wait_for_service(timeout_sec=5.0))
        finally:
            self.node.destroy_client(client)

    def test_request_without_padflie_execute_server_is_rejected(self):
        client = self.create_pad_right_client()
        try:
            self.assertTrue(client.wait_for_server(timeout_sec=5.0))
            send_future = client.send_goal_async(self.create_goal())
            self.assertTrue(self.spin_until(send_future.done))
            self.assertFalse(send_future.result().accepted)
        finally:
            client.destroy()

    def test_request_with_padflie_execute_server_completes(self):
        release = threading.Event()
        execute_called = threading.Event()
        acquired = threading.Event()

        def execute(goal_handle):
            execute_called.set()
            if not release.wait(timeout=5.0):
                result = PadExecute.Result()
                result.result = PadExecute.Result.RESULT_FAILURE
                goal_handle.abort()
                return result
            result = PadExecute.Result()
            result.result = PadExecute.Result.RESULT_ON_PAD
            goal_handle.succeed()
            return result

        server = ActionServer(
            self.node,
            PadExecute,
            'padflie1/pad_execute',
            execute_callback=execute,
            callback_group=ReentrantCallbackGroup(),
        )
        client = self.create_pad_right_client()

        def feedback(message):
            if (
                message.feedback.status
                == PadRightControl.Feedback.STATUS_ACQUIRED_RIGHT
            ):
                acquired.set()
                release.set()

        try:
            self.assertTrue(client.wait_for_server(timeout_sec=5.0))
            send_future = client.send_goal_async(
                self.create_goal(), feedback_callback=feedback
            )
            self.assertTrue(self.spin_until(send_future.done))
            goal_handle = send_future.result()
            self.assertTrue(goal_handle.accepted)

            result_future = goal_handle.get_result_async()
            self.assertTrue(self.spin_until(result_future.done))
            self.assertTrue(execute_called.is_set())
            self.assertTrue(acquired.is_set())
            self.assertTrue(result_future.result().result.success)
        finally:
            release.set()
            client.destroy()
            server.destroy()


@launch_testing.post_shutdown_test()
class TestSmartPadShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
