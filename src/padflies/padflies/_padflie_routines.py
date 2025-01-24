from copy import deepcopy
import numpy as np

from typing import List, Callable

from ._padflie_states import PadFlieState


def takeoff_routine(self, position: List[float]):
    self.target = position
    self.target[2] += 1.0

    self.state = PadFlieState.TAKEOFF
    self.hl_commander.takeoff(target_height=self.target[2], duration_seconds=4.0)
    self.sleep(2.0)
    self.state = PadFlieState.TARGET


def land_routine(
    self,
    get_pad_position: Callable[[], List[float]],
    get_position: Callable[[], List[float]],
):
    """
    Phase 1:
        For at most 8 seconds.
        Use cmd_positions to target a point 0.5 meters above the pad.
        If position is reached continue with Phase2.

        If is is not reached PROBLEM TODO

        During this we need to update the pad position, maybe it moves.
    """
    self.state = PadFlieState.TARGET_INTERNAL
    timeout = 8.0
    while timeout > 0.0:
        self.target = deepcopy(get_pad_position())
        self.target[2] += 0.5

        if (
            np.linalg.norm(np.array(get_position()) - np.array(self.target)) < 0.1
        ):  # closer than 10 cm
            break

        timeout -= 0.1
        self.sleep(0.1)

    self.state = PadFlieState.LAND
    self.ll_commander.notify_setpoints_stop(100)

    """
    Phase2: 
        We are now only using HighLevelCommander
        Use GoTos to properly land. 
        We were 0.5 meters above the pad. 
        First Step: lower to 0.2 m above pad. This is because Phase1 probably overshoots
        Second Step: Fly straight down into the pad. For a good seating. Update Pad Positon alway.

        During this phase a moving pad is not good
    """

    pad_position = get_pad_position()
    self.hl_commander.go_to(
        pad_position[0],
        pad_position[1],
        pad_position[2] + 0.2,
        yaw=0.0,
        duration_seconds=2.0,
    )
    # TODO: Make yaw available
    self.sleep(3.0)  # bleed off momentum

    pad_position = get_pad_position()
    self.hl_commander.go_to(
        pad_position[0],
        pad_position[1],
        pad_position[2] - 0.1,
        yaw=0.0,
        duration_seconds=4.0,
    )
    self.sleep(3.0)

    """
    Phase3: 
        We are already in the Pad. 
        We land to stop motors and this Wiggles us to properly seat into the pad.
    """
    self.hl_commander.land(
        target_height=0.0, duration_seconds=3.0
    )  # Try to get into the pad (landing at pad_height would be more correct)
    self.sleep(2.5)
    self.state = PadFlieState.IDLE
