# import wpilib
from commands2 import CommandBase
# import math


class SwerveDriveCommand(CommandBase):
    def __init__(self, swerve_subsystem, get_x, get_y, get_rot):
        """
        :param swerve_subsystem: instance of SwerveDriveSubsystem
        :param get_x: function returning joystick x input (-1 to 1)
        :param get_y: function returning joystick y input (-1 to 1)
        :param get_rot: function returning joystick rotation input (-1 to 1)
        """
        super().__init__()
        self.swerve = swerve_subsystem
        self.get_x = get_x
        self.get_y = get_y
        self.get_rot = get_rot

        self.deadband = 0.05  # same deadband as Java example

        # Declare subsystem dependencies
        self.addRequirements([self.swerve])

    def execute(self):
        x_speed = self.apply_deadband(self.get_x())
        y_speed = self.apply_deadband(self.get_y())
        rot = self.apply_deadband(self.get_rot())

        # Negate y input because joystick forward is usually negative
        y_speed = -y_speed

        # Max speed constants from your DriveConstants
        max_speed = self.swerve.kMaxSpeedMetersPerSecond
        max_angular_velocity = (
            self.swerve.kMaxAngularVelocity
        )  # e.g., radians per second

        # Scale joystick inputs to speeds
        x_speed *= max_speed
        y_speed *= max_speed
        rot *= max_angular_velocity

        # Call your subsystem drive method
        self.swerve.drive(
            x_speed, y_speed, rot, field_relative=True, is_open_loop=False
        )

    def apply_deadband(self, value: float) -> float:
        if abs(value) > self.deadband:
            if value > 0:
                return (value - self.deadband) / (1 - self.deadband)
            else:
                return (value + self.deadband) / (1 - self.deadband)
        else:
            return 0.0

    def isFinished(self):
        # This command never finishes on its own; runs until interrupted
        return False
