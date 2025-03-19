from enum import Enum, auto


class PadFlieState(Enum):
    DEACTIVATED = auto()
    IDLE = auto()
    TAKEOFF = auto()
    LAND = auto()
    TARGET = auto()
