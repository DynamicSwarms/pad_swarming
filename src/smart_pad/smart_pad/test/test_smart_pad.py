import unittest

from launch import LaunchDescription

from launch_ros.actions import Node

import launch_testing

import rclpy

from smart_pad_interfaces.srv import Lock


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
        cls.client = cls.node.create_client(Lock, 'smart_pad_0/lock')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_lock_service(self):
        # Wait for the service to be available
        self.assertTrue(self.client.wait_for_service(timeout_sec=5.0), 'Lock service not available')

        # Create a request to lock the pad
        request = Lock.Request()
        request.name = 'test_client'
        request.locking = True

        # Call the service and wait for the response
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)

        # Check that the service call was successful
        self.assertTrue(future.result().success, 'Failed to lock the pad')
        
@launch_testing.post_shutdown_test()
class TestSmartPadShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        # Check that all processes exited with code 0
        launch_testing.asserts.assertExitCodes(proc_info)