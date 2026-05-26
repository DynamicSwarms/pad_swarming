import threading
import time

import unittest
from unittest import result

from launch import LaunchDescription

from launch_ros.actions import Node

import launch_testing

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse

from smart_pad_interfaces.srv import Lock
from pad_management_interfaces.msg import PadInfo
from pad_management_interfaces.action import PadRightControl, PadExecute


def generate_test_description():
    smart_pads = []
    smart_pad_tfs = []
    for i in range(2):
        smart_pads.append(
            Node(
                package='pad_management_cpp',
                executable='pad_right_provider',
                name=f'smart_pad_{i}',
                output='screen',
                parameters=[
                    {
                        'pad_resource_manager_plugin': 'smart_pad::SmartPadResourceManager',
                        'id': i
                    }
                ]
            )
        )

        smart_pad_tfs.append(
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                arguments=[
                    '--x', f'{1.0 + i*0.5}',
                    '--y', '1.2',
                    '--z', '0.05',
                    '--yaw', '0',
                    '--pitch', '0',
                    '--roll', '0',
                    '--frame-id', 'world',
                    '--child-frame-id', f'smart_pad_{i}',
                ],
            )
        )

    return LaunchDescription([
        *smart_pads,
        *smart_pad_tfs,
        launch_testing.actions.ReadyToTest(),
    ])


class TestLockService(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        # Create a service client for the Lock service
        rclpy.init()
        cls.node = rclpy.create_node('test_lock_service_client')
        cls.info_sub = cls.node.create_subscription(
            PadInfo,
            'pad_management/pad_info',
            cls.pad_info_callback,
            rclpy.qos.QoSProfile(depth=10, reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT)
        )

        cls.infos_received = []

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def pad_info_callback(cls, msg):
        cls.infos_received.append(msg.pad_right_control_action_name)

    def test_lock_service_available(self):
        # Wait for the service to be available
        client = self.node.create_client(Lock, 'smart_pad_0/lock')
        self.assertTrue(client.wait_for_service(timeout_sec=5.0), 'Lock service not available')

    def test_pad_right_control_action(self):
        # Wait for the action server to be available
        action_client = ActionClient(
            self.node,
            PadRightControl,
            'smart_pad_0/pad_right_control'
        )
        self.assertTrue(action_client.wait_for_server(timeout_sec=5.0), 'PadRightControl action server not available')

        pad_right_goal = PadRightControl.Goal()
        pad_right_goal.name = "padflie1"
        pad_right_goal.action = PadRightControl.Goal.ACTION_TAKEOFF


        future = action_client.send_goal_async(pad_right_goal)
        rclpy.spin_until_future_complete(self.node, future)
        goal_handle = future.result()
        self.assertTrue(goal_handle is not None, 'Failed to send goal to PadRightControl action server')
        self.assertFalse(goal_handle.accepted, 'Goal should be rejected, since we dont have execute client')

    def test_pad_right_control_action_with_execute_client(self):
        action_client = ActionClient(
            self.node,
            PadRightControl,
            'smart_pad_0/pad_right_control',
            callback_group=rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        )

        state = {
            "goal_received": False,
            "cancel_received": False,
            "executed": False,
        }
        def goal_cb(goal_request):
            state.update({"goal_received": True})
            return GoalResponse.ACCEPT
        
        def cancel_cb(goal_handle):
            state.update({"cancel_received": True})
            return CancelResponse.ACCEPT
        

        def execute_cb(goal_handle):
            state["executed"] = True

            time.sleep(0.5)  
            result = PadExecute.Result()
            result.result = PadExecute.Result.RESULT_ON_PAD
            goal_handle.succeed()
            return result
        
        action_server = ActionServer(
            self.node,
            PadExecute,
            'padflie1/pad_execute',
            execute_callback=execute_cb,
            goal_callback=goal_cb,
            cancel_callback=cancel_cb,
            callback_group=rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        )
        self.assertTrue(action_client.wait_for_server(timeout_sec=5.0), 'PadRightControl action server not available')


        def feedback_cb(feedback_msg):
            if feedback_msg.feedback.status == PadRightControl.Feedback.STATUS_ACQUIRED_RIGHT:
                result = PadExecute.Result()
                result.result = PadExecute.Result.RESULT_ON_PAD


        pad_right_goal = PadRightControl.Goal()
        pad_right_goal.name = "padflie1"
        pad_right_goal.action = PadRightControl.Goal.ACTION_TAKEOFF
        send_goal_future = action_client.send_goal_async(pad_right_goal, feedback_callback=feedback_cb)
        rclpy.spin_until_future_complete(self.node, send_goal_future)
        goal_handle = send_goal_future.result()
        self.assertTrue(goal_handle is not None, 'Failed to send goal to PadRightControl action server')
        self.assertTrue(goal_handle.accepted, 'Goal should be accepted, since we have execute client')
        
        def right_control_result_cb(future):
            result = future.result().result
            self.assertTrue(result.success, 'PadRightControl action did not succeed')

            self.assertTrue(state["goal_received"], 'Pad Execute callback was not called')
            self.assertTrue(state["executed"], 'Pad Execute callback was not called')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(right_control_result_cb)

        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=1.0)
        self.assertTrue(result_future.result() is not None, 'Failed to get result from PadRightControl action server')
        print("done")
      

    def test_all_info_received(self):
        # Wait for some time to receive pad info messages
        rclpy.spin_once(self.node, timeout_sec=5.0)
        self.assertGreaterEqual(len(self.infos_received), 1, 'Did not receive any PadInfo messages')
    
        
@launch_testing.post_shutdown_test()
class TestSmartPadShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        # Check that all processes exited with code 0
        launch_testing.asserts.assertExitCodes(proc_info)