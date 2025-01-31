import numpy as np


class YawCommander:
    def __init__(self, dt: float, max_rotational_speed: float):
        """Initializes the yaw commander.

        Usage: call set_target_yaw() with the desired angle.
        The return value is the target you can safely send to the crazyflie.

        Args:
            dt (float): The timedelta the safe_cmd_yaw function is called
            max_rotational_speed (float): The maximum angle difference in radians/second.
        """
        self._dt = dt
        self._max_rotational_speed_tick = (
            max_rotational_speed * dt
        )  # Maximum rotational speed per tick (tick is one dt)

    def safe_cmd_yaw(self, current_yaw: float, target_yaw: float) -> float:
        """Set a target yaw and get yaw to command.

        Call this regularly in your update function.

        Args:
            target_yaw (float): The yaw target to get to

        Returns:
            float: The yaw command you can safly sent to crazyflie
        """
        target_yaw = self._normalize_angle(target_yaw)
        shortest_angle = self._shortest_angle(source=current_yaw, target=target_yaw)
        if abs(shortest_angle) > self._max_rotational_speed_tick:
            shortest_angle = np.sign(shortest_angle) * self._max_rotational_speed_tick

        return self._normalize_angle(current_yaw + shortest_angle)

    def _normalize_angle(self, angle: float) -> float:
        """Normalizes an angle to be in [-pi, pi]

        Args:
            angle (float): The angle to normalize (radians)

        Returns:
            float: The normalized angle (radians)
        """
        return np.arctan2(np.sin(angle), np.cos(angle))

    def _shortest_angle(self, source: float, target: float) -> float:
        """Computes the shortest angle between the source and target angle.

        The sign corresponds to direction.

        Args:
            source (float): The angle to start from
            target (float): The angle to go to

        Returns:
            float: The angle difference, sign is direction to go to.
        """
        return np.arctan2(np.sin(target - source), np.cos(target - source))
