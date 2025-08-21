import rclpy
from rclpy.node import Node
import rclpy.time
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rcl_interfaces.msg import ParameterDescriptor

from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import State as LifecycleState

from ament_index_python.packages import get_package_share_directory

import yaml
import numpy as np
from itertools import cycle
from threading import Lock
from .creator import Creator, BackendType


import tkinter as tk
from tkinter import scrolledtext
import threading

from typing import Callable


class GUI(threading.Thread):
    def __init__(self, flies, logger, add_callback: Callable[[int, int], None]):
        self.flies = flies
        self.logger = logger
        self.add_callback = add_callback
        threading.Thread.__init__(self)
        self.start()

    def close(self):
        self.root.quit()

    def run(self):
        self.root = tk.Tk()
        self.root.title("CrazyFlie Creator")

        self.create_all_btn = tk.Button(
            self.root, text="Create All", command=self.create_all
        )
        self.create_all_btn.pack()

        self.buttons = {}
        for flie in self.flies:
            cf_id = flie["id"]
            cf_channel = flie["channel"]

            btn = tk.Button(
                self.root,
                text=f"Connect {cf_id} (Ch {cf_channel})",
                command=lambda id=cf_id, ch=cf_channel: self.create_one(id, ch),
            )
            btn.pack()
            self.buttons[cf_id] = btn

        self.root.after(100, self.process_ui)
        self.root.mainloop()

    def process_ui(self):
        self.root.after(100, self.process_ui)

    def create_all(self):
        for flie in self.flies:
            self.create_one(flie["id"], flie["channel"])

    def create_one(self, cf_id, cf_channel):
        self.add_callback(cf_id, cf_channel)


class GUICreator(Node):
    """Manages the creation of crazyflies."""

    def __init__(self):
        super().__init__("creator")
        self.crazyflies_callback_group = (
            MutuallyExclusiveCallbackGroup()
        )  # Listening to transition Events

        # Declare parameters
        self.declare_parameter(
            name="setup_yaml",
            value=get_package_share_directory("pad_management")
            + "/config/lighthouse_config.yaml",
            descriptor=ParameterDescriptor(read_only=True),
        )
        self.declare_parameter(
            name="backend",
            value="hardware",
            descriptor=ParameterDescriptor(read_only=True),
        )
        # Read parameters
        yaml_file = self.get_parameter("setup_yaml").get_parameter_value().string_value

        # Load flies from yaml
        flies: list = []
        file = open(yaml_file, "r")
        flies += yaml.safe_load(file)["flies"]

        # Randomly permutate to add them in new order every time the system gets restarted.
        self.flie_list = flies
        self.flies = cycle(np.random.permutation(flies))

        if (
            self.get_parameter("backend").get_parameter_value().string_value
            == "hardware"
        ):
            self.adder = Creator(
                self,
                BackendType.HARDWARE,
                self.on_add_callback,
                self.on_failure_callback,
            )
        else:
            self.adder = Creator(
                self,
                BackendType.WEBOTS,
                self.on_add_callback,
                self.on_failure_callback,
            )

        self.added: list[int] = []
        self.added_lock: Lock = Lock()

    def add_crazyflie(self, cf_id: int, cf_channel: int):
        with self.added_lock:
            if cf_id in self.added:
                return  # We already try to create.

            # Add the crazyflie to be created by creator
            self.adder.enqueue_creation(cf_id, cf_channel=cf_channel)
            self.added.append(cf_id)

    def on_add_callback(self, cf_id: int, success: bool, msg: str):
        do_transition = False
        with self.added_lock:
            if success:
                do_transition = True
            else:
                if cf_id in self.added:
                    self.added.remove(cf_id)  # Retrry creation

        if do_transition:
            self._transition_padflie(
                cf_id=cf_id,
                state=LifecycleState.TRANSITION_STATE_CONFIGURING,
                label="configure",
            )

    def on_failure_callback(self, cf_id: int):
        do_transition = False
        with self.added_lock:
            if cf_id in self.added:
                self.added.remove(cf_id)
                do_transition = True
        if do_transition:
            self._transition_padflie(
                cf_id=cf_id,
                state=LifecycleState.TRANSITION_STATE_DEACTIVATING,
                label="deactivate",
            )  # Trie this
            self._transition_padflie(
                cf_id=cf_id,
                state=LifecycleState.TRANSITION_STATE_CLEANINGUP,
                label="cleanup",
            )  # But especially return to unconfigured

    def _transition_padflie(self, cf_id: int, state: LifecycleState, label: str):
        change_state_client = self.create_client(
            srv_type=ChangeState,
            srv_name=f"padflie{cf_id}/change_state",
            callback_group=self.crazyflies_callback_group,
        )
        if not change_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"The Padflie with ID {cf_id} is not available.")
            return  ## This permanently disables this crazyflie.

        request = ChangeState.Request()
        request.transition.id = state
        request.transition.label = label
        change_state_client.call_async(request)


def main():
    rclpy.init()

    creator = GUICreator()
    gui = GUI(creator.flie_list, creator.get_logger(), creator.add_crazyflie)

    executor = (
        MultiThreadedExecutor()
    )  # Because we are calling a service inside a timer
    try:
        while rclpy.ok():
            rclpy.spin_once(node=creator, timeout_sec=0.1, executor=executor)
        rclpy.try_shutdown()

    except KeyboardInterrupt:
        pass
    gui.close()
    gui.join()


if __name__ == "__main__":
    main()
