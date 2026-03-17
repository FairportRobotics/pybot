from wpilib import RobotBase
import math

ROBOT_MODE = "REAL" if RobotBase.isReal() else "SIM"

# Should we correct for the controller deadband?
CONTROLLER_CORRECT_FOR_DEADBAND = True
# The deadband for the controller joysticks
CONTROLLER_DEADBAND = 0.3
# The port the controller is connected to
CONTROLLER_PORT = 0

LED_LENGTH = 10
LED_PWM_PORT = 0

# Drivetrain physical constants
WHEEL_DIAMETER_IN_METERS = 0.1016  # 4-inch wheel
DRIVE_GEAR_RATIO = 6.75  # L2 MK4i ratio — adjust for your module
STEER_GEAR_RATIO = 21.4286  # MK4i steer ratio
METERS_PER_ROTATION = (math.pi * WHEEL_DIAMETER_IN_METERS) / DRIVE_GEAR_RATIO
MAX_SPEED_MPS = 4.5  # used for percent-output fallback
# Swerve module positions relative to robot center (meters)
# Positive X = forward, positive Y = left  (WPILib convention)
TRACK_WIDTH = 0.52  # left-to-right wheel center distance in meters
WHEEL_BASE = 0.52  # front-to-rear wheel center distance in meters

CAN_IDS = {
    "front_left_drive": 11,
    "front_left_steer": 12,
    "front_left_cancoder": 13,
    "front_right_drive": 21,
    "front_right_steer": 22,
    "front_right_cancoder": 23,
    "rear_left_drive": 31,
    "rear_left_steer": 32,
    "rear_left_cancoder": 33,
    "rear_right_drive": 41,
    "rear_right_steer": 42,
    "rear_right_cancoder": 43,
}

CANCODER_OFFSETS = {
    "front_left": 0.0,
    "front_right": 0.0,
    "rear_left": 0.0,
    "rear_right": 0.0,
}

PID = {
    "drive": {"P": 0.1, "I": 0.0, "D": 0.0},
    "steer": {"P": 40.0, "I": 0.0, "D": 0.5},
}


DRIVE_V = 0.12  # volts per RPS (feedforward)
