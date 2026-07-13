import threading
import unittest

from action_msgs.msg import GoalStatus
import launch_testing
from megapad_test_support import MegapadTestCase
from megapad_test_support import megapad_launch_description
from pad_management_interfaces.action import PadRightControl


def generate_test_description():
    return megapad_launch_description('megapad_holder_cancel_container')


class TestHolderCancel(MegapadTestCase):
    node_name = 'test_megapad_holder_cancel'

    def test_canceling_holder_allows_waiting_client_to_acquire(self):
        client_ids = (20, 21)
        releases = {client_id: threading.Event() for client_id in client_ids}

        servers = [
            self.create_releasable_execute_server(i, releases[i])
            for i in client_ids
        ]
        clients = [
            self.create_action_client()
            for _ in client_ids
        ]
        acquired = set()
        waiting = set()

        def feedback(client_id):
            def callback(message):
                if (
                    message.feedback.status
                    == PadRightControl.Feedback.STATUS_ACQUIRED_RIGHT
                ):
                    acquired.add(client_id)
                elif (
                    message.feedback.status
                    == PadRightControl.Feedback.STATUS_WAITING_FOR_RIGHT
                ):
                    waiting.add(client_id)
            return callback

        def send(index):
            client_id = client_ids[index]
            return self.send_request(
                clients[index], client_id, feedback(client_id)
            )

        try:
            self.assertTrue(clients[0].wait_for_server(timeout_sec=5.0))
            holder = send(0)
            self.assertTrue(self.spin_until(lambda: client_ids[0] in acquired))

            successor = send(1)
            self.assertTrue(self.spin_until(lambda: client_ids[1] in waiting))

            cancel_future = holder.cancel_goal_async()
            self.assertTrue(self.spin_until(cancel_future.done))
            self.assertEqual(len(cancel_future.result().goals_canceling), 1)
            canceled_result = holder.get_result_async()
            self.assertTrue(self.spin_until(canceled_result.done))
            self.assertEqual(
                canceled_result.result().status, GoalStatus.STATUS_CANCELED
            )

            self.assertTrue(self.spin_until(lambda: client_ids[1] in acquired))
            releases[client_ids[1]].set()
            successor_result = successor.get_result_async()
            self.assertTrue(self.spin_until(successor_result.done))
            self.assertTrue(successor_result.result().result.success)
        finally:
            for release in releases.values():
                release.set()
            for client in clients:
                client.destroy()
            for server in servers:
                server.destroy()


@launch_testing.post_shutdown_test()
class TestMegapadShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
