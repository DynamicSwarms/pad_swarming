import threading
import unittest

import launch_testing
from megapad_test_support import MegapadTestCase
from megapad_test_support import megapad_launch_description
from pad_management_interfaces.action import PadRightControl
import rclpy


CLIENT_COUNT = 5


def generate_test_description():
    return megapad_launch_description(
        'megapad_test_container', max_requests=CLIENT_COUNT
    )


class TestSimultaneousAccess(MegapadTestCase):
    node_name = 'test_megapad_simultaneous_access'
    executor_threads = CLIENT_COUNT + 2

    def test_five_clients_are_served_one_at_a_time(self):
        release_events = [threading.Event() for _ in range(CLIENT_COUNT)]
        execute_servers = [
            self.create_releasable_execute_server(i, release_events[i])
            for i in range(CLIENT_COUNT)
        ]
        action_clients = [
            self.create_action_client()
            for _ in range(CLIENT_COUNT)
        ]

        try:
            self.assertTrue(
                action_clients[0].wait_for_server(timeout_sec=5.0),
                'Megapad action server was not available',
            )

            acquired = set()
            acquired_lock = threading.Lock()

            def feedback_callback(client_id):
                def callback(feedback_message):
                    if (
                        feedback_message.feedback.status
                        == PadRightControl.Feedback.STATUS_ACQUIRED_RIGHT
                    ):
                        with acquired_lock:
                            acquired.add(client_id)

                return callback

            send_futures = []
            for client_id, action_client in enumerate(action_clients):
                goal = PadRightControl.Goal()
                goal.name = f'padflie{client_id}'
                goal.action = PadRightControl.Goal.ACTION_TAKEOFF
                goal.max_wait_time = rclpy.duration.Duration(seconds=10.0).to_msg()
                goal.usage_time = rclpy.duration.Duration(seconds=1.5).to_msg()
                send_futures.append(
                    action_client.send_goal_async(
                        goal,
                        feedback_callback=feedback_callback(client_id),
                    )
                )

            self.assertTrue(
                self.spin_until(
                    lambda: all(future.done() for future in send_futures), 5.0
                ),
                'Not all five goals received a response',
            )
            goal_handles = [future.result() for future in send_futures]
            self.assertTrue(
                all(handle is not None and handle.accepted
                    for handle in goal_handles),
                'The megapad did not accept all five requests',
            )

            completed = set()
            result_futures = [
                handle.get_result_async() for handle in goal_handles
            ]
            for _ in range(CLIENT_COUNT):
                self.assertTrue(
                    self.spin_until(
                        lambda: len(acquired - completed) == 1, 5.0
                    ),
                    'Exactly one waiting client did not acquire the megapad',
                )
                with acquired_lock:
                    current_holders = acquired - completed
                    self.assertEqual(
                        len(current_holders),
                        1,
                        'More than one client acquired the megapad',
                    )
                    holder = next(iter(current_holders))

                release_events[holder].set()
                self.assertTrue(
                    self.spin_until(lambda: result_futures[holder].done(), 5.0),
                    f'padflie{holder} did not finish',
                )
                self.assertTrue(result_futures[holder].result().result.success)
                completed.add(holder)

            self.assertEqual(len(acquired), CLIENT_COUNT)
        finally:
            for action_client in action_clients:
                action_client.destroy()
            for execute_server in execute_servers:
                execute_server.destroy()

@launch_testing.post_shutdown_test()
class TestMegapadShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
