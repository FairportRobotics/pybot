import os
from enum import Enum
from wpilib import RobotBase


class RobotModes(Enum):
    REAL = 1
    SIMULATION = 2
    REPLAY = 3


SIM_MODE = (
    RobotModes.REPLAY
    if "LOG_PATH" in os.environ and os.environ["LOG_PATH"] != ""
    else RobotModes.SIMULATION
)
ROBOT_MODE = RobotModes.REAL if RobotBase.isReal() else SIM_MODE

# Should we correct for the controller deadband?
CONTROLLER_CORRECT_FOR_DEADBAND = True
# The deadband for the controller joysticks
CONTROLLER_DEADBAND = 0.3
# The port the controller is connected to
CONTROLLER_PORT = 0

LED_LENGTH = 10
LED_PWM_PORT = 0
