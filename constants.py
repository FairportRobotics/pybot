from wpilib import RobotBase

ROBOT_MODE = "REAL" if RobotBase.isReal() else "SIM"

# Should we correct for the controller deadband?
CONTROLLER_CORRECT_FOR_DEADBAND = True
# The deadband for the controller joysticks
CONTROLLER_DEADBAND = 0.3
# The port the controller is connected to
CONTROLLER_PORT = 0

LED_LENGTH = 10
LED_PWM_PORT = 0
