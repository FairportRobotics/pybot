import components
import constants
import magicbot

class MyRobot(magicbot.MagicRobot):
    LED: components.LED

    def createObjects(self) -> None:
        '''Create motors and stuff here'''
        self.LED_length = constants.LED_LENGTH
        self.LED_pwm_port = constants.LED_PWM_PORT

    def teleopInit(self) -> None:
        '''Called when teleop starts; optional'''
        pass

    def teleopPeriodic(self) -> None:
        self.LED.rainbow()

    def disabledInit(self) -> None:
        pass

    def disabledPeriodic(self) -> None:
        self.LED.turn_off()