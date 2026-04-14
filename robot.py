import components
import constants
from magicbot import MagicRobot


class MyRobot(MagicRobot):
    controller: components.XboxController
    drivetrain: components.TankDrive

    def createObjects(self):
        """Create motors and stuff here"""
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

    def disabledInit(self):
        pass

    def disabledPeriodic(self):
        pass
