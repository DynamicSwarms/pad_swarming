import threading
import unittest

import launch_testing
from pad_management_interfaces.action import PadExecute, PadRightControl
from megapad_test_support import MegapadTestCase
from megapad_test_support import megapad_launch_description


def generate_test_description():
    return megapad_launch_description(
        'megapad_repeated_access_container'
    )


class TestRepeatedAccess(MegapadTestCase):
    node_name = 'test_megapad_repeated_access'
    executor_threads = 3

    def test_same_padflie_can_acquire_twice(self):
        client_id = 30
        release = threading.Event()
        execution_count = 0

        def execute(goal_handle):
            nonlocal execution_count
            execution_count += 1
            expected_execution = execution_count
            if not release.wait(timeout=10.0):
                result = PadExecute.Result()
                result.result = PadExecute.Result.RESULT_FAILURE
                result.reason = 'Test timed out waiting for release'
                goal_handle.abort()
                return result
            release.clear()
            result = PadExecute.Result()
            result.result = PadExecute.Result.RESULT_ON_PAD
            result.reason = f'Completed execution {expected_execution}'
            goal_handle.succeed()
            return result

        server = self.create_execute_server(client_id, execute)
        client = self.create_action_client()

        try:
            self.assertTrue(client.wait_for_server(timeout_sec=5.0))
            for expected_access_count in (1, 2):
                acquired = threading.Event()

                def feedback(message):
                    if (
                        message.feedback.status
                        == PadRightControl.Feedback.STATUS_ACQUIRED_RIGHT
                    ):
                        acquired.set()

                goal_handle = self.send_request(
                    client, client_id, feedback
                )
                self.assertTrue(self.spin_until(acquired.is_set))

                release.set()
                result_future = goal_handle.get_result_async()
                self.assertTrue(self.spin_until(result_future.done))
                self.assertTrue(result_future.result().result.success)
                self.assertEqual(execution_count, expected_access_count)
        finally:
            release.set()
            client.destroy()
            server.destroy()


@launch_testing.post_shutdown_test()
class TestMegapadShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
