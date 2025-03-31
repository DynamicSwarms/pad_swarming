import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from rclpy.time import Time, Duration
from threading import Lock

from pad_management_interfaces.srv import PadRightAcquire, PadRightRelease


class TrafficController(Node):

    def __init__(self):
        super().__init__("traffic_controller")

        self.max_acquire_time = Duration(seconds=30, nanoseconds=0)

        self.pad_lock: Lock = Lock()
        self.locked_name: str = ""
        self.locked_time: Time = self.get_clock().now()

        self.waitings_lock: Lock = Lock()
        self.waitings: "set[str]" = set()

        self.acquire_service = self.create_service(
            srv_type=PadRightAcquire,
            srv_name="acquire_pad_right",
            callback=self.acquire_callback,
            callback_group=ReentrantCallbackGroup(),
        )

        self.release_group = MutuallyExclusiveCallbackGroup()

        self.release_service = self.create_service(
            srv_type=PadRightRelease,
            srv_name="release_pad_right",
            callback=self.release_callback,
            callback_group=self.release_group,
        )

        self.create_timer(1.0, self.release_timeout_timer)

    def acquire_callback(
        self, request: PadRightAcquire.Request, response: PadRightAcquire.Response
    ):
        """A crazyflie wants to acquire Pad rights.

        The pad however is locked with the pad_lock.
        Wait until this is free. Then grant.
        But, do not wait longer than some timeout.
        Args:
            request (Trigger.Request): the name of the person who locked. and a timeout
            response (Trigger.Response): has a boolean success -> true if Pad right granted
        """
        name = request.name
        self.waitings.add(name)

        timeout: float = request.timeout
        while name in self.waitings and timeout > 0.0:
            timeout -= 0.3
            if self.pad_lock.acquire(timeout=0.3):
                if name in self.waitings:
                    self.waitings.remove(request.name)

                self.locked_name = request.name
                self.locked_time = self.get_clock().now()
                response.success = True
                return response

        if name in self.waitings:
            self.waitings.remove(request.name)

        response.success = False
        return response

    def release_callback(
        self, request: PadRightRelease.Request, response: PadRightRelease.Response
    ):
        """A crazyflie wants to relaese Pad rights.

        First check if the lock is locked by the crazyflie asking.
        Then release.
        Args:
            request (PadRightRelease.Request): includes the name
            response (PadRightRelease.Response): success if the action was successfull
        """
        name = request.name
        if name in self.waitings:
            self.waitings.remove(name)

        if self.pad_lock.locked() and request.name == self.locked_name:
            self.pad_lock.release()
            response.success = True
            return response

        response.success = False
        return response

    def release_timeout_timer(self):
        now = self.get_clock().now()
        if self.pad_lock.locked() and self.locked_time + self.max_acquire_time < now:
            self.pad_lock.release()
            self.get_logger().info(
                f"Lock timed out. Releasing anyway. Current holder: {self.locked_name}"
            )


def main():
    rclpy.init()
    tc = TrafficController()
    executor = MultiThreadedExecutor()
    executor.add_node(tc)
    try:
        executor.spin()
        rclpy.try_shutdown()
    except KeyboardInterrupt:
        rclpy.try_shutdown()
    exit()


if __name__ == "__main__":
    main()
