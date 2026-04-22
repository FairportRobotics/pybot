class Controller:
    # Should we correct for the controller deadband?
    CORRECT_FOR_DEADBAND = True
    # The deadband for the controller joysticks
    DEADBAND = 0.3
    # The port the controller is connected to
    PORT = 0


class CAN_IDs:
    # The CAN ID for the left drive motor
    LEFT_MOTOR = 11
    LEFT_FOLLOWER_MOTOR = 12
    # The CAN ID for the right drive motor
    RIGHT_MOTOR = 21
    RIGHT_FOLLOWER_MOTOR = 22


class Robot:
    DISTANCE_BETWEEN_WHEELS_IN_INCHES = 24
    GEAR_RATIO = 10.71
    WHEEL_RADIUS_IN_INCHES = 4
    ENCODER_TICKS_PER_ROTATION = 4096
    MASS_IN_LBS = 100
    MASS_IN_KG = MASS_IN_LBS * 0.453592
    MOMENT_OF_INERTIA = 6.0  # kg·m², typical value for a 24" wide robot
