"""Larger and less symmetric scenarios for reciprocal collision avoidance."""

import math
import random

import rclpy
from rclpy.executors import MultiThreadedExecutor

import collision_avoidance_examples.velocity_reciprocal_test as base_test
from collision_avoidance_examples.velocity_reciprocal_test import Agent


def circle_scenario(agent_count, radius=4.0, speed=0.65):
    """Place agents on a circle and send each one through its center."""
    agents = []
    for index in range(agent_count):
        angle = 2.0 * math.pi * index / agent_count
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)
        agents.append(
            Agent(
                [x, y],
                [-speed * math.cos(angle), -speed * math.sin(angle)],
            )
        )
    return agents


def random_crossing(agent_count, seed, area=4.0, speed=0.65):
    """Create a repeatable random crowd whose agents cross the test area."""
    generator = random.Random(seed)
    agents = []

    for _ in range(agent_count):
        start_angle = generator.uniform(0.0, 2.0 * math.pi)
        start_radius = generator.uniform(0.75 * area, area)
        position = [
            start_radius * math.cos(start_angle),
            start_radius * math.sin(start_angle),
        ]

        # Aim near, but deliberately not exactly at, the opposite side.
        target_angle = start_angle + math.pi + generator.uniform(-0.45, 0.45)
        target = [
            area * math.cos(target_angle) + generator.uniform(-0.4, 0.4),
            area * math.sin(target_angle) + generator.uniform(-0.4, 0.4),
        ]
        dx = target[0] - position[0]
        dy = target[1] - position[1]
        length = math.hypot(dx, dy)
        agent_speed = generator.uniform(0.65 * speed, speed)
        agents.append(
            Agent(
                position,
                [agent_speed * dx / length, agent_speed * dy / length],
                radius=generator.uniform(0.22, 0.35),
                max_speed=1.0,
            )
        )

    return agents


# Add or edit scenarios here. Keep the keys unique: they are used by the
# ``scenario`` ROS parameter on the command line.
EXTENDED_SCENARIOS = {
    # Four opposing lanes with small offsets, unequal speeds, and imperfect
    # alignment. Eight agents total.
    "offset_lanes_8": [
        Agent([-4.0, -1.15], [0.72, 0.03]),
        Agent([-3.7, -0.35], [0.61, -0.02]),
        Agent([-4.2, 0.42], [0.68, 0.01]),
        Agent([-3.8, 1.22], [0.57, -0.04]),
        Agent([4.1, -1.05], [-0.63, -0.02]),
        Agent([3.8, -0.28], [-0.70, 0.03]),
        Agent([4.2, 0.50], [-0.59, -0.01]),
        Agent([3.9, 1.12], [-0.66, 0.04]),
    ],
    # Two slightly skewed flows crossing at the center. Ten agents total.
    "skewed_crossing_10": [
        Agent([-4.0, -1.2], [0.68, 0.10]),
        Agent([-4.2, -0.5], [0.72, 0.05]),
        Agent([-3.8, 0.2], [0.62, -0.03]),
        Agent([-4.1, 0.9], [0.66, -0.08]),
        Agent([-3.9, 1.5], [0.58, -0.12]),
        Agent([-1.4, -4.0], [0.12, 0.64]),
        Agent([-0.7, -4.2], [0.07, 0.71]),
        Agent([0.0, -3.8], [-0.02, 0.60]),
        Agent([0.8, -4.1], [-0.08, 0.68]),
        Agent([1.5, -3.9], [-0.13, 0.57]),
    ],
    "circle_8": circle_scenario(8),
    "circle_12": circle_scenario(12),
    "random_8": random_crossing(8, seed=8),
    "random_12": random_crossing(12, seed=12),
    "random_20": random_crossing(20, seed=20, area=5.0, speed=0.7),
}


def main(args=None):
    # The reusable visualizer reads its scenarios from this module-level map.
    # Replacing it here keeps velocity_reciprocal_test.py unchanged.
    base_test.SCENARIOS = EXTENDED_SCENARIOS

    rclpy.init(args=args)
    node = base_test.VelocityReciprocalTest()
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
