import threading
import time
import unittest

from launch import LaunchDescription
import launch
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
import launch_testing
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
import rclpy
from padflies_interfaces.msg import AvailabilityInfo
from std_srvs.srv import Trigger

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
                    '--x', f'{1.0 - i*0.4}',
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
        executable='component_container',
        arguments=['--executor-type', 'multi-threaded'],
        output='screen',
        composable_node_descriptions=smart_pads,
    )

    gateway = Node(
        package='crazyflie_simulation_gateway',
        executable='gateway',
        output='screen',
        sigterm_timeout='10.0',
        parameters=[{'use_sim_time': True}],
    )

    spawner = Node(
        package='crazyflie_simulation_examples',
        executable='crazyflie_spawner',
        output='screen',
        parameters=[{'count': 1}],
    )

    padflie = Node(
        package='padflies_cpp',
        executable='padflie',
        name='padflie0',
        parameters=[
            {
                'id': 0,
                'use_sim_time': True,
            }
        ],
    )

    sim_clock = Node(
        package='crazyflie_simulation_examples',
        executable='clock',
        output='screen',
        parameters=[{'rate': 10.0}],
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


class TestPadflieSimulation(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_padflie')
        cls.executor = rclpy.executors.MultiThreadedExecutor()
        cls.executor.add_node(cls.node)

        callback_group = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        cls.availability_subscription = cls.node.create_subscription(
            AvailabilityInfo,
            '/availability',
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

        cls.spin_thread.join(timeout=2.0)
        rclpy.shutdown()

    @classmethod
    def availability_callback(cls, msg):
        if msg.name == 'padflie0':
            cls.padflie_available.set()

    @classmethod
    def wait_for_future(cls, future, timeout_sec=1.0):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if future.done():
                return True
            time.sleep(0.1)
        return False

    def activate_deactivate_padflie(self, activate: bool, wait_time_sec=1.0):
        client = self.node.create_client(
            ChangeState,
            '/padflie0/change_state'
        )

        client.wait_for_service(timeout_sec=0.1)
        self.assertTrue(
            client.service_is_ready(),
            'ChangeState service for /padflie0 not available',
        )

        req = ChangeState.Request()
        req.transition.id = (
            Transition.TRANSITION_ACTIVATE
            if activate else Transition.TRANSITION_DEACTIVATE
        )

        future = client.call_async(req)
        self.assertTrue(
            self.wait_for_future(future, timeout_sec=wait_time_sec),
            'ChangeState service call for /padflie0 timed out',
        )
        self.assertIsNotNone(future.result())
        self.assertTrue(future.result().success)
        self.node.destroy_client(client)

    def test_padflie_lifecycle(self):
        client = self.node.create_client(GetState, '/padflie0/get_state')
        self.assertTrue(client.wait_for_service(timeout_sec=5.0))

        configured = False
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            future = client.call_async(GetState.Request())
            if self.wait_for_future(future) and future.result() is not None:
                if future.result().current_state.id == State.PRIMARY_STATE_INACTIVE:
                    configured = True
                    break

        self.node.destroy_client(client)
        self.assertTrue(configured, 'padflie did not reach the inactive state')

    def test_padflie_activation(self):
        self.assertTrue(self.padflie_available.wait(timeout=5.0))

        self.activate_deactivate_padflie(activate=True)
        self.activate_deactivate_padflie(activate=False)

        self.padflie_available.clear()
  
    
    def test_padflie_takeoff(self):
        self.assertTrue(self.padflie_available.wait(timeout=5.0))
        self.activate_deactivate_padflie(activate=True)

        takeoff_client = self.node.create_client(Trigger, '/padflie0/takeoff')
        takeoff_client.wait_for_service(timeout_sec=0.1)
        self.assertTrue(takeoff_client.service_is_ready())
        req = Trigger.Request()
        future = takeoff_client.call_async(req)
        self.assertTrue(self.wait_for_future(future, timeout_sec=10.0))
        self.assertIsNotNone(future.result())
        self.assertTrue(future.result().success)
        self.node.destroy_client(takeoff_client)

        self.activate_deactivate_padflie(
            activate=False, wait_time_sec=10.0
        )
        self.padflie_available.clear()

@launch_testing.post_shutdown_test()
class TestSmartPadShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
