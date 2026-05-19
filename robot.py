from components import LED
import constants
from magicbot import MagicRobot, feedback
import wpilib


class MyRobot(MagicRobot):
    led_strip: LED

    def createObjects(self) -> None:
        """Create motors and stuff here"""
        self.accelerometer = wpilib.BuiltInAccelerometer()
        self.led_strip_length = constants.LED.LENGTH
        self.led_strip_pwm_port = constants.LED.PWM_PORT

    def teleopPeriodic(self) -> None:
        self.led_strip.rainbow()

    def disabledPeriodic(self) -> None:
        self.led_strip.turn_off()

    @feedback
    def accelerometer_x(self) -> float:
        return self.accelerometer.getX()

    @feedback
    def accelerometer_y(self) -> float:
        return self.accelerometer.getY()
