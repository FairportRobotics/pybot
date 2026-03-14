import components
import constants
from magicbot import MagicRobot


class MyRobot(MagicRobot):
    ACCELEROMETER: components.RoboRioAccelerometer
    GYRO: components.NavX2
    LED: components.LED
    MAIN_CONTROLLER: components.XboxController
    SCRIBE: components.Scribe

    def createObjects(self):
        """Create motors and stuff here"""
        # Controller stuff here
        self.MAIN_CONTROLLER_CORRECT_FOR_DEADBAND = (
            constants.CONTROLLER_CORRECT_FOR_DEADBAND
        )
        self.MAIN_CONTROLLER_DEADBAND = constants.CONTROLLER_DEADBAND
        self.MAIN_CONTROLLER_PORT = constants.CONTROLLER_PORT
        # LED stuff here
        self.LED_LENGTH = constants.LED_LENGTH
        self.LED_PWM_PORT = constants.LED_PWM_PORT
        self.LED_DEFAULT_COLOR = "red"
        self.LED_DEFAULT_MODE = "solid"

    def teleopInit(self):
        """Called when teleop starts; optional"""
        pass

    def teleopPeriodic(self):
        self.MAIN_CONTROLLER.capture_button_presses()
        left_x, left_y, right_x, right_y = self.MAIN_CONTROLLER.get_joysticks()

    def disabledInit(self):
        pass

    def disabledPeriodic(self):
        self.LED.turn_off()
