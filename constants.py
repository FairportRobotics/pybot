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


class TankDrive:
    # Current limit for drivetrain motors. 60A is a reasonable maximum to reduce
    # likelihood of tripping breakers or damaging CIM motors
    DRIVE_MOTOR_CURRENT_LIMIT = 60
