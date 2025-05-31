import components
import constants
import magicbot
import wpilib

if constants.MODE == "simulation":
    import os

    os.environ["HALSIMXRP_HOST"] = "192.168.42.1"
    os.environ["HALSIMXRP_PORT"] = "3540"


class Robot(magicbot.MagicRobot):
    # List all the components used by the robot
    climber: components.Climber
    controller: components.XboxController
    # Set up the drive based on the drivetrain type specified in constants.py
    drivetrain_type = constants.DRIVETRAIN_TYPE.lower()  # Ensure it's lowercase
    if drivetrain_type == "arcade":
        drive: components.ArcadeDrive
    elif drivetrain_type == "tank":
        drive: components.TankDrive
    elif drivetrain_type == "swerve":
        drive: components.SwerveDrive
    else:
        raise ValueError(f"Unknown drivetrain type: {drivetrain_type}")
    elevator: components.Elevator
    feeder: components.Feeder
    intake: components.Intake
    shooter: components.Shooter

    # List what we want to see in the network tables

    def createObjects(self):
        # Controller
        self.xbox_controller = wpilib.XboxController(constants.CONTROLLER_PORT)

        # Drivetrain motors
        self.left_front_motor = wpilib.PWMSparkMax(constants.LEFT_FRONT_MOTOR_PORT)
        self.left_back_motor = wpilib.PWMSparkMax(constants.LEFT_BACK_MOTOR_PORT)
        self.right_front_motor = wpilib.PWMSparkMax(constants.RIGHT_FRONT_MOTOR_PORT)
        self.right_back_motor = wpilib.PWMSparkMax(constants.RIGHT_BACK_MOTOR_PORT)
        # Let's group the motors together as a single "motor"
        self.left_motor = wpilib.MotorControllerGroup(
            self.left_front_motor, self.left_back_motor
        )
        self.right_motor = wpilib.MotorControllerGroup(
            self.right_front_motor, self.right_back_motor
        )
        # Invert left side so it goes forward when the joystick is pushed forward
        self.left_motor.setInverted(True)

        # Elevator Heights
        self.elevator_levels = constants.ELEVATOR_LEVELS

    def teleopPeriodic(self):
        # Get the input from the controller
        left_x, left_y, right_x, right_y = self.controller.get_joysticks()
        if self.drivetrain_type in ["arcade", "tank"]:
            self.drive.go(left_y, right_y)
        elif self.drivetrain_type == "swerve":
            self.drive.go(left_x, left_y, right_x)

        if self.controller.dpad_up():
            self.elevator.move_to(self.elevator.get_current_level() + 1)

        elif self.controller.dpad_down():
            self.elevator.move_to(self.elevator.get_current_level() - 1)


if __name__ == "__main__":
    wpilib.run(Robot)
