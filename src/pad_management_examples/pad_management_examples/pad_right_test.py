import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from pad_management_interfaces.action import PadRightControl


class PadRightActionClient(Node):
    def __init__(self):
        super().__init__("pad_right_action_client")
        self._action_client = ActionClient(self, PadRightControl, "pad_right_control")

        self._id = self.declare_parameter("id", 0).get_parameter_value().integer_value
        self._hold_time = (
            self.declare_parameter("hold_time", 5.0).get_parameter_value().double_value
        )

        self._cancel_timer = self.create_timer(self._hold_time, self.cancel_goal)

    def cancel_goal(self):
        future = self._goal_handle.cancel_goal_async()
        future.add_done_callback(self.cancel_done)
        self._cancel_timer.cancel()

    def cancel_done(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info("Goal successfully canceled.")
        else:
            self.get_logger().info("Goal failed to cancel.")

    def send_goal(self):
        padright_msg = PadRightControl.Goal()
        padright_msg.name = f"cf{self._id}"
        padright_msg.max_wait_time = 10.0
        padright_msg.usage_time = 5.0

        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Action server not available after waiting")
            return

        self._send_goal_future = self._action_client.send_goal_async(
            padright_msg, feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected")
            return
        self._goal_handle = goal_handle

        self.get_logger().info("Goal accepted")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Result was received, with success: {result.success}")

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        status = feedback.status
        status_str = next(
            (
                name
                for name, value in PadRightControl.Feedback.__dict__.items()
                if value == status
            ),
            str(status),
        )

        self.get_logger().info(
            f"Feedback received: {status_str}, time remaining: {feedback.time_remaining}"
        )


def main():
    rclpy.init()
    action_client = PadRightActionClient()
    action_client.send_goal()

    try:
        rclpy.spin(action_client)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
