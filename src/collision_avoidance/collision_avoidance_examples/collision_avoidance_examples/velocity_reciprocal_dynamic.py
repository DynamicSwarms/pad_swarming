"""Scenarios with smoothly changing preferred velocities."""

import math
import random

import rclpy
from rclpy.executors import MultiThreadedExecutor

import collision_avoidance_examples.velocity_reciprocal_test as base_test
from collision_avoidance_examples.velocity_reciprocal_test import Agent


def ring(agent_count, radius=4.0):
    agents = []
    for index in range(agent_count):
        angle = 2.0 * math.pi * index / agent_count
        agents.append(
            Agent(
                [radius * math.cos(angle), radius * math.sin(angle)],
                [-0.6 * math.cos(angle), -0.6 * math.sin(angle)],
            )
        )
    return agents


def random_agents(agent_count, seed, area=4.0):
    generator = random.Random(seed)
    agents = []
    for _ in range(agent_count):
        angle = generator.uniform(0.0, 2.0 * math.pi)
        distance = generator.uniform(0.6 * area, area)
        heading = angle + math.pi + generator.uniform(-0.4, 0.4)
        agents.append(
            Agent(
                [distance * math.cos(angle), distance * math.sin(angle)],
                [0.6 * math.cos(heading), 0.6 * math.sin(heading)],
                radius=generator.uniform(0.22, 0.32),
            )
        )
    return agents


DYNAMIC_SCENARIOS = {
    # Two opposing groups weave sideways while moving through each other.
    "weaving_8": [
        Agent([-4.0, -1.5], [0.65, 0.0]),
        Agent([-4.0, -0.5], [0.65, 0.0]),
        Agent([-4.0, 0.5], [0.65, 0.0]),
        Agent([-4.0, 1.5], [0.65, 0.0]),
        Agent([4.0, -1.4], [-0.65, 0.0]),
        Agent([4.0, -0.4], [-0.65, 0.0]),
        Agent([4.0, 0.6], [-0.65, 0.0]),
        Agent([4.0, 1.6], [-0.65, 0.0]),
    ],
    # Agents continue toward the opposite side, but their desired speed pulses.
    "pulsing_circle_10": ring(10),
    # The desired destination itself slowly rotates around the center.
    "rotating_targets_12": ring(12, radius=4.5),
    # Repeatable crowds whose headings meander continuously.
    "smooth_random_8": random_agents(8, seed=108),
    "smooth_random_12": random_agents(12, seed=112),
    "smooth_random_20": random_agents(20, seed=120, area=5.0),
}


class DynamicVelocityTest(base_test.VelocityReciprocalTest):
    def update(self):
        name = self.scenario_names[self.scenario_index]
        self.update_preferred_velocities(name, self.elapsed)
        super().update()

    def update_preferred_velocities(self, name, time):
        if name == "weaving_8":
            self.update_weaving(time)
        elif name == "pulsing_circle_10":
            self.update_pulsing_circle(time)
        elif name == "rotating_targets_12":
            self.update_rotating_targets(time)
        elif name.startswith("smooth_random_"):
            self.update_smooth_random(time)

    def update_weaving(self, time):
        for index, agent in enumerate(self.agents):
            direction = 1.0 if index < 4 else -1.0
            phase = index * 0.7
            agent.preferred_velocity = [
                direction * (0.60 + 0.08 * math.sin(0.35 * time + phase)),
                0.20 * math.sin(0.65 * time + phase),
            ]

    def update_pulsing_circle(self, time):
        for index, agent in enumerate(self.agents):
            distance = math.hypot(*agent.position)
            if distance < 1.0e-6:
                continue
            phase = 2.0 * math.pi * index / len(self.agents)
            speed = 0.48 + 0.20 * math.sin(0.45 * time + phase)
            agent.preferred_velocity = [
                -speed * agent.position[0] / distance,
                -speed * agent.position[1] / distance,
            ]

    def update_rotating_targets(self, time):
        target_radius = 1.4
        for index, agent in enumerate(self.agents):
            phase = 2.0 * math.pi * index / len(self.agents)
            target_angle = phase + math.pi + 0.28 * time
            target_x = target_radius * math.cos(target_angle)
            target_y = target_radius * math.sin(target_angle)
            dx = target_x - agent.position[0]
            dy = target_y - agent.position[1]
            distance = max(math.hypot(dx, dy), 1.0e-6)
            speed = 0.62
            agent.preferred_velocity = [
                speed * dx / distance,
                speed * dy / distance,
            ]

    def update_smooth_random(self, time):
        for index, agent in enumerate(self.agents):
            # Two low-frequency waves produce deterministic, smooth wandering.
            phase = index * 1.618
            heading = (
                phase
                + 0.38 * time
                + 0.75 * math.sin(0.21 * time + phase)
                + 0.25 * math.sin(0.47 * time + 0.3 * phase)
            )
            speed = 0.48 + 0.14 * math.sin(0.31 * time + phase)
            agent.preferred_velocity = [
                speed * math.cos(heading),
                speed * math.sin(heading),
            ]


def main(args=None):
    base_test.SCENARIOS = DYNAMIC_SCENARIOS

    rclpy.init(args=args)
    node = DynamicVelocityTest()
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
