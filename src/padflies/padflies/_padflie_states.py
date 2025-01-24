from enum import Enum, auto


class PadFlieState(Enum):
    IDLE = auto()
    TAKEOFF = auto()
    LAND = auto()
    TARGET = auto()
    TARGET_INTERNAL = auto()  # Do not accept external targets. But flie with target
