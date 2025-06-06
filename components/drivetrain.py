import magicbot
import wpilib
import wpilib.drive


class ArcadeDrive:
    left_motor: wpilib.MotorControllerGroup
    right_motor: wpilib.MotorControllerGroup
    speed = magicbot.tunable(0.0)

    def execute(self) -> None:
        pass

    def setup(self) -> None:
        """
        Set up the arcade drive with the specified motors.
        """
        self.drive = wpilib.drive.DifferentialDrive(self.left_motor, self.right_motor)

    def go(self, forward: float, rotation: float) -> None:
        """
        Move the arcade drive in a specific direction.

        :param forward: The forward speed.
        :param rotation: The rotation speed.
        """
        self.drive.arcadeDrive(forward, rotation, squareInputs=True)

    def stop(self) -> None:
        self.drive.stopMotor()


class SwerveDrive:
    speed = magicbot.tunable(0.0)

    def execute(self) -> None:
        pass

    def go(self, x: float, y: float, rotation: float) -> None:
        """
        Move the swerve drive in a specific direction.

        :param x: The x component of the movement vector.
        :param y: The y component of the movement vector.
        :param rotation: The rotation speed.
        """
        pass

    def stop(self) -> None:
        pass


class TankDrive:
    left_motor: wpilib.MotorControllerGroup
    right_motor: wpilib.MotorControllerGroup
    speed = magicbot.tunable(0.0)

    def execute(self) -> None:
        pass

    def setup(self) -> None:
        """
        Set up the arcade drive with the specified motors.
        """
        self.drive = wpilib.drive.DifferentialDrive(self.left_motor, self.right_motor)

    def go(self, left: float, right: float) -> None:
        """
        Move the tank drive with left and right motor speeds.

        :param left: The speed for the left motors.
        :param right: The speed for the right motors.
        """
        pass

    def stop(self) -> None:
        pass
