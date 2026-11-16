import threading
import unittest

from action_msgs.msg import GoalStatus
import launch_testing
from megapad_test_support import MegapadTestCase
from megapad_test_support import megapad_launch_description
from pad_management_interfaces.action import PadRightControl


CLIENT_IDS = (10, 11)


def generate_test_description():
    return megapad_launch_description('megapad_cancel_test_container')


class TestPreemptiveCancel(MegapadTestCase):
    node_name = 'test_megapad_preemptive_cancel'

    def test_waiting_client_can_cancel_before_acquiring(self):
        release_events = {
            client_id: threading.Event() for client_id in CLIENT_IDS
        }
        execute_servers = [
            self.create_releasable_execute_server(
                client_id, release_events[client_id]
            )
            for client_id in CLIENT_IDS
        ]
        action_clients = [
            self.create_action_client()
            for _ in CLIENT_IDS
        ]
        acquired = set()
        waiting = set()

        def feedback_callback(client_id):
            def callback(feedback_message):
                status = feedback_message.feedback.status
                if status == PadRightControl.Feedback.STATUS_ACQUIRED_RIGHT:
                    acquired.add(client_id)
                elif status == PadRightControl.Feedback.STATUS_WAITING_FOR_RIGHT:
                    waiting.add(client_id)
            return callback

        def send_request(index):
            client_id = CLIENT_IDS[index]
            return self.send_request(
                action_clients[index],
                client_id,
                feedback_callback(client_id),
                usage_seconds=2.0,
            )

        try:
            self.assertTrue(action_clients[0].wait_for_server(timeout_sec=5.0))

            holder_handle = send_request(0)
            self.assertTrue(
                self.spin_until(lambda: CLIENT_IDS[0] in acquired, 5.0)
            )

            waiting_handle = send_request(1)
            self.assertTrue(
                self.spin_until(lambda: CLIENT_IDS[1] in waiting, 5.0)
            )
            self.assertNotIn(CLIENT_IDS[1], acquired)

            cancel_future = waiting_handle.cancel_goal_async()
            self.assertTrue(self.spin_until(cancel_future.done, 5.0))
            self.assertEqual(len(cancel_future.result().goals_canceling), 1)

            canceled_result = waiting_handle.get_result_async()
            self.assertTrue(self.spin_until(canceled_result.done, 5.0))
            self.assertEqual(
                canceled_result.result().status, GoalStatus.STATUS_CANCELED
            )
            self.assertNotIn(CLIENT_IDS[1], acquired)

            release_events[CLIENT_IDS[0]].set()
            holder_result = holder_handle.get_result_async()
            self.assertTrue(self.spin_until(holder_result.done, 5.0))
            self.assertTrue(holder_result.result().result.success)
            self.assertNotIn(CLIENT_IDS[1], acquired)
        finally:
            for event in release_events.values():
                event.set()
            for action_client in action_clients:
                action_client.destroy()
            for execute_server in execute_servers:
                execute_server.destroy()


@launch_testing.post_shutdown_test()
class TestMegapadShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
