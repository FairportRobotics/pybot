import components
import constants
import magicbot


class MyRobot(magicbot.MagicRobot):
    CONTROLLER: components.XboxController
    LED: components.LED
    LOGGER: components.Lumberjack

    def createObjects(self):
        """Create motors and stuff here"""
        # Controller stuff here
        self.CONTROLLER_CORRECT_FOR_DEADBAND = constants.CONTROLLER_CORRECT_FOR_DEADBAND
        self.CONTROLLER_DEADBAND = constants.CONTROLLER_DEADBAND
        self.CONTROLLER_PORT = constants.CONTROLLER_PORT
        # LED stuff here
        self.LED_length = constants.LED_LENGTH
        self.LED_pwm_port = constants.LED_PWM_PORT

    def teleopInit(self):
        """Called when teleop starts; optional"""
        pass

    def teleopPeriodic(self):
        self.CONTROLLER.capture_button_presses()
        left_x, left_y, right_x, right_y = self.CONTROLLER.get_joysticks()

    def disabledInit(self):
        pass

    def disabledPeriodic(self):
        self.LED.turn_off()
