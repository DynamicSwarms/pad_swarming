import math
from dataclasses import dataclass

import rclpy
from collision_avoidance_interfaces.srv import (
    VelocityReciprocalsCollisionAvoidance,
)
from geometry_msgs.msg import Point
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


@dataclass
class Agent:
    position: list[float]
    preferred_velocity: list[float]
    radius: float = 0.3
    max_speed: float = 1.0


SCENARIOS = {
    "head_on": [
        Agent([-3.0, 0.0], [0.6, 0.0]),
        Agent([3.0, 0.0], [-0.6, 0.0]),
    ],
    "crossing": [
        Agent([-3.0, 0.0], [0.6, 0.0]),
        Agent([0.0, -3.0], [0.0, 0.6]),
    ],
    "parallel": [
        Agent([-3.0, -0.7], [0.6, 0.0]),
        Agent([-3.0, 0.7], [0.6, 0.0]),
    ],
    "overtaking": [
        Agent([-3.0, 0.0], [0.8, 0.0]),
        Agent([-1.0, 0.0], [0.3, 0.0]),
    ],
    "stationary": [
        Agent([-3.0, 0.0], [0.6, 0.0]),
        Agent([0.0, 0.0], [0.0, 0.0]),
    ],
    "three_way": [
        Agent([-3.0, 0.0], [0.6, 0.0]),
        Agent([1.5, -2.6], [-0.3, 0.52]),
        Agent([1.5, 2.6], [-0.3, -0.52]),
    ],
}


class VelocityReciprocalTest(Node):
    def __init__(self):
        super().__init__("velocity_reciprocal_test")

        self.declare_parameter("scenario", "all")
        self.declare_parameter("scenario_duration", 12.0)
        self.declare_parameter("time_step", 0.1)

        requested_scenario = self.get_parameter("scenario").value
        if requested_scenario == "all":
            self.scenario_names = list(SCENARIOS)
        elif requested_scenario in SCENARIOS:
            self.scenario_names = [requested_scenario]
        else:
            valid = ", ".join(["all", *SCENARIOS])
            raise ValueError(
                f"Unknown scenario '{requested_scenario}'. Choose one of: {valid}"
            )

        self.dt = float(self.get_parameter("time_step").value)
        self.scenario_duration = float(
            self.get_parameter("scenario_duration").value
        )
        self.scenario_index = 0
        self.elapsed = 0.0
        self.agents: list[Agent] = []
        self.trails: list[list[Point]] = []

        self.timer_group = MutuallyExclusiveCallbackGroup()
        self.client_group = MutuallyExclusiveCallbackGroup()
        self.client = self.create_client(
            VelocityReciprocalsCollisionAvoidance,
            "/velocity_reciprocal_collision_avoidance",
            callback_group=self.client_group,
        )
        self.marker_publisher = self.create_publisher(
            MarkerArray, "visualization_marker_array", 10
        )

        self.get_logger().info("Waiting for reciprocal-velocity service...")
        while rclpy.ok() and not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available yet")

        self.load_scenario()
        self.timer = self.create_timer(
            self.dt, self.update, callback_group=self.timer_group
        )

    def load_scenario(self):
        name = self.scenario_names[self.scenario_index]
        self.agents = [
            Agent(
                agent.position.copy(),
                agent.preferred_velocity.copy(),
                agent.radius,
                agent.max_speed,
            )
            for agent in SCENARIOS[name]
        ]
        self.trails = [[] for _ in self.agents]
        self.elapsed = 0.0
        self.get_logger().info(f"Starting scenario: {name}")

    def update(self):
        if self.elapsed >= self.scenario_duration:
            self.scenario_index = (self.scenario_index + 1) % len(
                self.scenario_names
            )
            self.delete_markers()
            self.load_scenario()

        updated_velocities = []
        for agent_id, agent in enumerate(self.agents):
            velocity, collision = self.calculate_velocity(agent_id, agent)
            updated_velocities.append(velocity)
            if collision:
                self.get_logger().debug(
                    f"Agent {agent_id}: {agent.preferred_velocity} -> {velocity}"
                )

        # Advance all agents together after all service responses are available.
        markers = []
        for agent_id, (agent, velocity) in enumerate(
            zip(self.agents, updated_velocities)
        ):
            agent.position[0] += velocity[0] * self.dt
            agent.position[1] += velocity[1] * self.dt
            self.add_trail_point(agent_id, agent.position)
            markers.extend(self.agent_markers(agent_id, agent, velocity))
            markers.append(self.trail_marker(agent_id))

        self.marker_publisher.publish(MarkerArray(markers=markers))

        self.elapsed += self.dt

    def calculate_velocity(self, agent_id, agent):
        request = VelocityReciprocalsCollisionAvoidance.Request()
        request.id = agent_id
        request.position.x = agent.position[0]
        request.position.y = agent.position[1]
        request.velocity.x = agent.preferred_velocity[0]
        request.velocity.y = agent.preferred_velocity[1]
        request.radius = agent.radius
        request.max_speed = agent.max_speed

        response = self.client.call(request)
        return [response.velocity.x, response.velocity.y], response.collision

    def agent_markers(self, agent_id, agent, velocity):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "agents"
        marker.id = agent_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = agent.position[0]
        marker.pose.position.y = agent.position[1]
        marker.pose.orientation.w = 1.0
        marker.scale.x = 2.0 * agent.radius
        marker.scale.y = 2.0 * agent.radius
        marker.scale.z = 2.0 * agent.radius
        marker.color.a = 1.0
        marker.color.r, marker.color.g, marker.color.b = self.color(agent_id)
        speed = math.hypot(*velocity)
        arrow = Marker()
        arrow.header = marker.header
        arrow.ns = "velocities"
        arrow.id = agent_id
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        start = Point(x=agent.position[0], y=agent.position[1], z=0.0)
        end = Point(
            x=agent.position[0] + velocity[0],
            y=agent.position[1] + velocity[1],
            z=0.0,
        )
        arrow.points = [start, end]
        arrow.scale.x = 0.05
        arrow.scale.y = 0.12
        arrow.scale.z = 0.12
        arrow.color.a = 1.0
        arrow.color.r, arrow.color.g, arrow.color.b = self.color(agent_id)
        if speed < 1.0e-6:
            arrow.action = Marker.DELETE
        return [marker, arrow]

    def add_trail_point(self, agent_id, position):
        trail = self.trails[agent_id]
        trail.append(Point(x=position[0], y=position[1], z=0.0))
        del trail[:-300]

    def trail_marker(self, agent_id):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "trails"
        marker.id = agent_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.03
        marker.color.a = 0.8
        marker.color.r, marker.color.g, marker.color.b = self.color(agent_id)
        marker.points = self.trails[agent_id]
        return marker

    def delete_markers(self):
        marker = Marker()
        marker.action = Marker.DELETEALL
        self.marker_publisher.publish(MarkerArray(markers=[marker]))

    @staticmethod
    def color(agent_id):
        colors = [
            (1.0, 0.2, 0.2),
            (0.2, 1.0, 0.2),
            (0.2, 0.4, 1.0),
            (1.0, 0.8, 0.2),
        ]
        return colors[agent_id % len(colors)]


def main(args=None):
    rclpy.init(args=args)
    node = VelocityReciprocalTest()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
