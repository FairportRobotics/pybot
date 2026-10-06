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
        self.huskylens_default_algorithm = constants.HUSKYLENS_DEFAULT_ALGORITHM

    def teleopPeriodic(self):
        self.led.set_mode("knightrider")

    def disabledPeriodic(self):
        self.led.turn_off()
