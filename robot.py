import components
import constants
import phoenix5
from wpimath.geometry import Pose2d
from magicbot import MagicRobot, feedback


class MyRobot(MagicRobot):
    controller: components.XboxController
    drivetrain: components.TankDrive

    def createObjects(self):
        """Create motors and stuff here"""
        # Motors
        self.drivetrain_motors = {
            "left_motor": phoenix5.WPI_TalonSRX(constants.CAN_IDs.LEFT_MOTOR),
            "left_follower": phoenix5.WPI_TalonSRX(constants.CAN_IDs.LEFT_FOLLOWER_MOTOR),
            "right_motor": phoenix5.WPI_TalonSRX(constants.CAN_IDs.RIGHT_MOTOR),
            "right_follower": phoenix5.WPI_TalonSRX(constants.CAN_IDs.RIGHT_FOLLOWER_MOTOR),
        }        

        # Controller stuff here
        self.controller_correct_for_deadband = constants.Controller.CORRECT_FOR_DEADBAND
        self.controller_deadband = constants.Controller.DEADBAND
        self.controller_port = constants.Controller.PORT

    def teleopInit(self):
        """Called when teleop starts; optional"""
        pass

    def teleopPeriodic(self):
        self.controller.capture_button_presses()
        left_x, left_y, right_x, right_y = self.controller.get_joysticks()
        self.drivetrain.drive(-left_y, right_x)

        if self.controller.x_button_was_pressed():
            self.drivetrain.reset()

    def disabledInit(self):
        pass

    def disabledPeriodic(self):
        pass

    @feedback
    def pose(self) -> Pose2d:
        return self.drivetrain.pose()
