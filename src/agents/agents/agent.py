import rclpy

from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.executors import MultiThreadedExecutor
from rcl_interfaces.msg import (
    ParameterDescriptor,
    SetParametersResult,
    FloatingPointRange,
)
import rclpy.parameter

from .agent_commander import AgentCommander
from .agent_connector import AgentConnector

from geometry_msgs.msg import PoseStamped
from typing import List

import time

from enum import Enum, auto

class AgentState(Enum):
    IDLE = auto()
    CONNECTING = auto()
    FLYING = auto()
    LANDING = auto()

class Agent(Node):

    def __init__(self):
        super().__init__("agent")
        self.declare_parameter(
            name="id", value=0, descriptor=ParameterDescriptor(read_only=True)
        )
        self.declare_parameter(name="active", value=False)
        self.add_on_set_parameters_callback(self.__set_parameter_callback)

        self._state = AgentState.IDLE

        self._commander = AgentCommander(node=self)
        self._connector = AgentConnector(node=self, on_connect_callback=self.on_connect, on_disconnect_callback=self.on_disconnect)

        self.create_timer(0.1, self.control_loop)

    def control_loop(self):
        if self._state == AgentState.FLYING: 
            target = PoseStamped()
            target.header.frame_id = "world"
            target.pose.position.z = 1.0 # Hover at 0, 0, 1 in world
            self._commander.send_target(target)
        if self._state == AgentState.LANDING:
            if self._commander.is_home():
                self._state = AgentState.IDLE
                self._connector.disconnect()


    def on_connect(self, prefix: str):
        """
        Connector module found a crazyflie to connect to.
        We connect the commander, takeoff and fly
        """
        self._commander.connect(prefix)
        time.sleep(0.3) # For Padflie to activate properly
        self._commander.takeoff()
        self._state = AgentState.FLYING
    
    def on_disconnect(self, prefix: str):
        """
        Connector module says its disconnected.
        Either because we landed and triggered it or there was an error in padflie.
        We disconnect the commander module and are ready to be connected again.        
        """
        self._commander.disconnect()
        if self._state == AgentState.FLYING:
            # We are flying. Get swapped for a new one.
            return
        else:  
            self._state = AgentState.IDLE
            self.__set_activate_param_false()

    def __param_activate_callback(self, shall_activate: bool) -> bool:
        if shall_activate: 
            if self._state == AgentState.IDLE: 
                self._connector.start_search()
                self._state = AgentState.CONNECTING
                return True
            else: # We are not in IDLE and will not activate.
                self.get_logger().info(f"Not IDLE ({self._state})-> Not activating.")
                return False
            
        else: # shall_deactivate
            if self._state == AgentState.FLYING:  
                self._commander.land()
                self._state = AgentState.LANDING
                return True
            elif self._state == AgentState.CONNECTING:
                self._connector.stop_search()
                self._state = AgentState.IDLE
                return True
            elif self._state == AgentState.IDLE:
                return True # Comming from a landing
            elif self._state == AgentState.LANDING: 
                # Already landing which will result in disconnect automatically.
                return True
            else:  
                self.get_logger().info(f"State is: {self._state}. Shouldnt happen??")
                return False

    def __set_parameter_callback(self, params: List[Parameter]) -> SetParametersResult:
        for param in params:
            if param.name == "active":
                success = self.__param_activate_callback(param.value)
                return SetParametersResult(successful=success)
        return SetParametersResult(successful=True)

    def __set_activate_param_false(self):
        param = rclpy.parameter.Parameter(
            "active", rclpy.Parameter.Type.BOOL, False
        )
        self.set_parameters([param])


def main():
    rclpy.init()

    agent = Agent()
    executor = MultiThreadedExecutor()
    executor.add_node(agent)
    executor.spin()

    rclpy.try_shutdown()
    exit()
if __name__ == "__main__":
    main()
