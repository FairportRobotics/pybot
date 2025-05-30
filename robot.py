import components
import constants
import magicbot
import wpilib

if constants.MODE == "simulation":
    import os

    os.environ["HALSIMXRP_HOST"] = "192.168.42.1"
    os.environ["HALSIMXRP_PORT"] = "3540"


class MyRobot(magicbot.MagicRobot):
    # List all the components used by the robot
    climber: components.Climber
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
        # This is where we initialize the motors and other hardware objects to pass into the components

        # Controller
        self.controller = wpilib.XboxController(constants.CONTROLLER_PORT)

        # Drivetrain motors
        self.left_front_motor = wpilib.PWMTalonSRX(constants.LEFT_FRONT_MOTOR_PORT)
        self.left_back_motor = wpilib.PWMTalonSRX(constants.LEFT_BACK_MOTOR_PORT)
        self.right_front_motor = wpilib.PWMTalonSRX(constants.RIGHT_FRONT_MOTOR_PORT)
        self.right_back_motor = wpilib.PWMTalonSRX(constants.RIGHT_BACK_MOTOR_PORT)

        self.elevator_levels = {0: 0, 1: 12, 2: 48, 3: 56}

    def teleopPeriodic(self):
        # Get the input from the controller
        self.left_joystick_y = round(self.controller.getLeftY(), 1)
        self.left_joystick_x = round(self.controller.getLeftX(), 1)
        self.right_joystick_x = round(self.controller.getRightX(), 1)
        self.right_joystick_y = round(self.controller.getRightY(), 1)
        if self.drivetrain_type == "arcade":
            self.drive.go(-self.left_joystick_y, -self.right_joystick_x)
        elif self.drivetrain_type == "tank":
            self.drive.go(-self.left_joystick_y, -self.right_joystick_y)
        elif self.drivetrain_type == "swerve":
            self.drive.go(
                -self.left_joystick_x, -self.left_joystick_y, -self.right_joystick_x
            )

        if self.controller.getPOV() == 0:
            self.elevator.move_to(self.elevator.get_level() + 1)

        elif self.controller.getPOV() == 180:
            self.elevator.move_to(self.elevator.get_level() - 1)
