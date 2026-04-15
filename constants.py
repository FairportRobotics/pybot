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
    DISTANCE_BETWEEN_WHEELS_INCHES = 24
    GEAR_RATIO = 10.71
    WHEEL_RADIUS_INCHES = 2
    ENCODER_TICKS_PER_ROTATION = 1024  # 4096
