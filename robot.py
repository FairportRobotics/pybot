import components
import constants
from magicbot import MagicRobot


class MyRobot(MagicRobot):
    huskylens: components.HuskyLens
    led: components.LED

    def createObjects(self):
        # LED stuff here
        self.led_length = constants.LED_LENGTH
        self.led_pwm_port = constants.LED_PWM_PORT

    def teleopPeriodic(self):
        pass

    def disabledPeriodic(self):
        self.led.turn_off()
