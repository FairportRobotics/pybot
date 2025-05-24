from enum import Enum, auto


class Mode(Enum):
    """Defines the operation modes for the robot."""

    REAL = auto()
    SIM = auto()
    REPLAY = auto()


# Current operating mode
CURRENT_MODE = Mode.REAL

# Driver controller ID
DRIVER_XBOX_CONTROLLER_ID = 0


class DriveConstants:
    LEFT_MAIN_TALON_ID = 37
    LEFT_FOLLOW_TALON_ID = 7
    RIGHT_MAIN_TALON_ID = 20
    RIGHT_FOLLOW_TALON_ID = 10


class ArmConstants:
    ARM_EXTENSION_SOLENOID_ID = 0
    ARM_RETRACTION_SOLENOID_ID = 1


# Pneumatics hub CAN ID
PH_CAN_ID = 0
