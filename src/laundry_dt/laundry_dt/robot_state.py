from enum import Enum

class RobotState(Enum):
    IDLE     = 0
    ADVANCING = 1
    LIFTING   = 2
    BACK_WINCHING = 3
    COMPLETE  = 4