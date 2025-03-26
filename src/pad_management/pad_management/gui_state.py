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
from tkinter import scrolledtext, ttk
import threading

from typing import Callable
from crazyflie_interfaces.msg import GenericLogData

from dataclasses import dataclass
from typing import Dict

import time


@dataclass
class UIElement:
    battery_progress: ttk.Progressbar
    battery_label: tk.Label
    charge_label: tk.Label
    state_label: tk.Label
    canFly: ttk.Progressbar
    isFlying: ttk.Progressbar
    isTumbled: ttk.Progressbar
    lastUpdateTime: "Time"


from enum import Enum


class ChargeState(Enum):
    BATTERY = 0
    CHARGING = 1
    CHARGED = 2
    LOWPOWER = 3
    SHUTDOWN = 4


class GUI_State(threading.Thread):
    def __init__(self, flies):
        self.flies = flies

        threading.Thread.__init__(self)
        self.start()

    def close(self):
        self.root.quit()

    def run(self):
        self.root = tk.Tk()
        self.root.title("CrazyFlie State Visualizer")

        self.elements: Dict[int, UIElement] = {}
        for flie in self.flies:
            cf_id = flie["id"]
            cf_channel = flie["channel"]

            frame = tk.Frame(self.root, relief=tk.RAISED, borderwidth=2)
            frame.pack(fill=tk.X, padx=5, pady=5)

            tk.Label(frame, text=f"ID: {cf_id} (0x{cf_id:X})").pack(
                side=tk.LEFT, padx=5
            )

            battery_progress = ttk.Progressbar(frame, length=100, mode="determinate")
            battery_progress.pack(side=tk.LEFT)

            battery_label = tk.Label(frame, text="- V")
            battery_label.pack(side=tk.LEFT)

            tk.Label(frame, text="Current:").pack(side=tk.LEFT, padx=5)
            charge_label = tk.Label(frame, text="- mA")
            charge_label.pack(side=tk.LEFT)

            state_label = tk.Label(frame, text="Unknown")
            state_label.pack(side=tk.LEFT)

            tk.Label(frame, text="CanFly:").pack(side=tk.LEFT, padx=5)
            canFly = ttk.Progressbar(
                frame,
                length=10,
                value=0,
            )
            canFly.pack(side=tk.LEFT)

            tk.Label(frame, text="isFlying:").pack(side=tk.LEFT, padx=5)
            isFlying = ttk.Progressbar(
                frame,
                length=10,
                mode="determinate",
                value=0,
            )
            isFlying.pack(side=tk.LEFT)

            tk.Label(frame, text="isTumbled:").pack(side=tk.LEFT, padx=5)
            isTumbled = ttk.Progressbar(
                frame,
                length=10,
                mode="determinate",
                value=0,
            )
            isTumbled.pack(side=tk.LEFT)

            self.elements[cf_id] = UIElement(
                battery_progress=battery_progress,
                battery_label=battery_label,
                charge_label=charge_label,
                state_label=state_label,
                canFly=canFly,
                isFlying=isFlying,
                isTumbled=isTumbled,
                lastUpdateTime=time.time(),
            )
        self.timeout_duration = 4.0
        self.root.after(100, self.process_ui)
        self.root.mainloop()

    def process_ui(self):
        current_time = time.time()
        for key in self.elements.keys():
            elm = self.elements[key]
            if current_time - elm.lastUpdateTime > self.timeout_duration:
                elm.battery_progress["value"] = 0
        self.root.after(100, self.process_ui)

    def update_state(self, cf_id: int, values: "list[float]"):
        voltage, charge_current, charge_state, canFly, isFlying, isTumbled = values
        charge_state = ChargeState(int(charge_state))
        canFly = bool(int(canFly))
        isFlying = bool(int(isFlying))
        isTumbled = bool(int(isTumbled))

        if cf_id in self.elements.keys():
            elm: UIElement = self.elements[cf_id]
            elm.battery_progress["value"] = max(0, min(100, (voltage / 4.2) * 100))
            elm.battery_label["text"] = "{:.2f} V".format(voltage)
            elm.charge_label["text"] = "{:.2f} mA".format(charge_current)
            elm.state_label["text"] = f"{charge_state}"

            def setState(pb: ttk.Progressbar, state: bool):
                if state:
                    pb["value"] = 100
                else:
                    pb["value"] = 0

            setState(elm.canFly, canFly)
            setState(elm.isFlying, isFlying)
            setState(elm.isTumbled, isTumbled)
            elm.lastUpdateTime = time.time()


class BatteryVisualizer(Node):
    """Displays state of crazyflies."""

    def __init__(self):
        super().__init__("statate_viz")
        self.gui_state_callback: Callable[[int, "list[float]"], None] = None

        # Declare parameters
        self.declare_parameter(
            name="setup_yaml",
            value=get_package_share_directory("pad_management")
            + "/config/lighthouse_config.yaml",
            descriptor=ParameterDescriptor(read_only=True),
        )
        # Read parameters
        yaml_file = self.get_parameter("setup_yaml").get_parameter_value().string_value

        # Load flies from yaml
        flies: list = []
        file = open(yaml_file, "r")
        flies += yaml.safe_load(file)["flies"]
        self.flies = flies

        for flie in self.flies:
            cf_id = flie["id"]
            cf_channel = flie["channel"]
            self.create_subscription(
                GenericLogData,
                f"/cf{cf_id}/state",
                lambda msg, c_id=cf_id, c_ch=cf_channel: self.state_callback(
                    msg, c_id, cf_channel
                ),
                qos_profile=10,
            )

    def state_callback(self, msg: GenericLogData, cf_id: int, cf_channel: int):
        if self.gui_state_callback is not None:
            self.gui_state_callback(cf_id, msg.values)


def main():
    rclpy.init()

    bv = BatteryVisualizer()
    gui = GUI_State(bv.flies)
    bv.gui_state_callback = gui.update_state

    executor = (
        MultiThreadedExecutor()
    )  # Because we are calling a service inside a timer
    try:
        while rclpy.ok():
            rclpy.spin_once(node=bv, timeout_sec=0.1, executor=executor)
        rclpy.try_shutdown()
    except KeyboardInterrupt:
        pass
    gui.close()
    gui.join()


if __name__ == "__main__":
    main()
