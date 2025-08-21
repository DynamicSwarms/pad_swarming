from enum import IntEnum, auto


class PadFlieState(IntEnum):
    DEACTIVATED = auto()
    IDLE = auto()
    TAKEOFF = auto()
    LAND = auto()
    TARGET = auto()
