import components
import constants
from magicbot import MagicRobot


class MyRobot(MagicRobot):
    xbox_controller: components.XboxController
    drivetrain: components.TankDrive

    def createObjects(self):
        """Create motors and stuff here"""
        # Controller stuff here
        self.xbox_controller_correct_for_deadband = (
            constants.Controller.CORRECT_FOR_DEADBAND
        )
        self.xbox_controller_deadband = constants.Controller.DEADBAND
        self.xbox_controller_port = constants.Controller.PORT

    def teleopInit(self):
        """Called when teleop starts; optional"""
        pass

    def teleopPeriodic(self):
        self.xbox_controller.capture_button_presses()
        left_x, left_y, right_x, right_y = self.xbox_controller.get_joysticks()

    def disabledInit(self):
        pass

    def disabledPeriodic(self):
        pass
