import threading
import time

import unittest
from unittest import result


from launch import LaunchDescription
import launch
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

import launch_testing

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse

from smart_pad_interfaces.srv import Lock
from pad_management_interfaces.msg import PadInfo
from pad_management_interfaces.action import PadRightControl, PadExecute
from lifecycle_msgs.srv import GetState, ChangeState
from lifecycle_msgs.msg import State, Transition
from std_srvs.srv import Trigger
from std_msgs.msg import String

def generate_test_description():
    smart_pads = []
    smart_pad_tfs = []
    for i in range(2):
        smart_pads.append(
            ComposableNode(
                package='pad_management_cpp',
                plugin='PadRightActionServerNode',
                name=f'smart_pad_{i}',
                parameters=[
                    {
                        'pad_resource_manager_plugin': 'smart_pad::SmartPadResourceManager',
                        'id': i,
                        'use_sim_time': True,
                    }
                ]
            )
        )
        smart_pad_tfs.append(
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                parameters=[{"use_sim_time": True}],
                arguments=[
                    '--x', f'{1.0 + i*0.4}',
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

    smart_pad_container = ComposableNodeContainer(
        name='smart_pad_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        composable_node_descriptions=smart_pads,
    )

    gateway = Node(
        package="crazyflie_simulation_gateway",
        executable="gateway",
        output="screen",
        sigterm_timeout="10.0",
        parameters=[{"use_sim_time": True}],
    )

    spawner = Node(
        package="crazyflie_simulation_examples",
        executable="crazyflie_spawner",
        output="screen",
        parameters=[{"count": 1}],
    )

    padflie = Node(
        package='padflies_cpp',
        executable='padflie',
        name='padflie0',
        #prefix=["gdbserver localhost:3000"],
        parameters=[
            {
                'id': 0,
                'pad_id': 0,
                'use_sim_time': True,
            }
        ],
    )

    sim_clock = Node(
        package="crazyflie_simulation_examples",
        executable="clock",
        output="screen",
        parameters=[{"rate": 10.0}],
    )



    return LaunchDescription([
        smart_pad_container,
        *smart_pad_tfs,
        gateway,
        spawner,
        padflie,
        sim_clock,
        launch.actions.TimerAction(
            period=2.0,
            actions=[launch_testing.actions.ReadyToTest()]
        ),
    ])


class TestLockService(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        # Create a service client for the Lock service
        rclpy.init()
        cls.node = rclpy.create_node('test_padflie')
        cls.executor = rclpy.executors.MultiThreadedExecutor()
        cls.executor.add_node(cls.node)

        callback_group = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        cls.node.create_subscription(
            String, 
            "/availability",
            cls.availability_callback,
            rclpy.qos.QoSProfile(depth=10),
            callback_group=callback_group
        )
        cls.padflie_available = threading.Event()

        cls.spin_thread = threading.Thread(
            target=cls.executor.spin,
            daemon=True
        )
        cls.spin_thread.start()


       
    @classmethod
    def tearDownClass(cls):
        cls.executor.remove_node(cls.node)
        cls.executor.shutdown(wait_for_threads=True)
        cls.node.destroy_node()

        del cls.executor
        rclpy.shutdown()

    @classmethod
    def availability_callback(cls, msg):
        if msg.data == "padflie0":
            cls.padflie_available.set()

    @classmethod
    def wait_for_future(cls, future, timeout_sec=1.0):
        start_time = time.time()
        while time.time() - start_time < timeout_sec:
            if future.done():
                return True
            time.sleep(0.1)
        return False

    def activate_deactivate_padflie(self, activate: bool):
        client = self.node.create_client(
            ChangeState,
            '/padflie0/change_state'
        )

        client.wait_for_service(timeout_sec=0.1)
        self.assertTrue(client.service_is_ready(), "ChangeState service for /padflie0 not available")

        req = ChangeState.Request()
        req.transition.id = Transition.TRANSITION_ACTIVATE if activate else Transition.TRANSITION_DEACTIVATE

        future = client.call_async(req)
        self.assertTrue(self.wait_for_future(future, timeout_sec=1.0), "ChangeState service call for /padflie0 timed out")                   

        self.assertTrue(future.done(), "ChangeState service for /padflie0 timed out")
        self.assertTrue(future.result() is not None, "No result from ChangeState service for /padflie0")
        self.assertTrue(future.result().success, "Failed to activate padflie")

#    def test_padflie_lifecycle(self):
#        # Wait at most 5 seconds for padflie lifecycle to reach ACTIVE or INACTIVE
#        client = self.node.create_client(GetState, '/padflie0/get_state')
#        start_time = time.time()
#        timeout = 5.0
#        # wait for service availability
#        client.wait_for_service(timeout_sec=1.0)
#        self.assertTrue(client.service_is_ready(), "GetState service for /padflie0 not available")
#
#        configured = False
#        while time.time() - start_time < timeout:
#            try:
#                req = GetState.Request()
#                future = client.call_async(req)
#                rclpy.spin_until_future_complete(self.node, future, timeout_sec=0.1, executor=self.executor)
#                if future.done() and future.result() is not None:
#                    state = future.result().current_state
#                    if state.id is State.PRIMARY_STATE_INACTIVE:
#                        configured = True
#                        break
#            except Exception:
#                pass
#            time.sleep(0.1)
#
#        self.assertTrue(configured, "padflie lifecycle not in INACTIVE/ACTIVE within timeout")
#
    def test_padflie_activation(self):
        self.assertTrue(self.padflie_available.wait(timeout=1.0), "padflie0 did not become available within timeout")

        self.activate_deactivate_padflie(activate=True)
        self.activate_deactivate_padflie(activate=False)

        self.padflie_available.clear()
  
    
    def test_padflie_takeoff(self):
        self.assertTrue(self.padflie_available.wait(timeout=1.0), "padflie0 did not become available within timeout")
        self.activate_deactivate_padflie(activate=True)

        takeoff_client = self.node.create_client(Trigger, '/padflie0/takeoff')
        takeoff_client.wait_for_service(timeout_sec=0.1)
        self.assertTrue(takeoff_client.service_is_ready(), "Trigger service for /padflie0 not available")
        req = Trigger.Request()
        future = takeoff_client.call_async(req)
        self.assertTrue(self.wait_for_future(future, timeout_sec=10.0), "Trigger service call for /padflie0 timed out")

        self.assertTrue(future.done() and future.result() is not None, "Failed to call Trigger service for /padflie0")
        self.assertTrue(future.result().success, "Failed to initiate takeoff for padflie")

        self.activate_deactivate_padflie(activate=False)
        self.padflie_available.clear()

@launch_testing.post_shutdown_test()
class TestSmartPadShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        # Check that all processes exited with code 0
        launch_testing.asserts.assertExitCodes(proc_info)