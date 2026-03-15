import components
import constants
from magicbot import MagicRobot


class MyRobot(MagicRobot):
    scribe: components.Scribe
    accelerometer: components.RoboRioAccelerometer
    gyro: components.NavX2
    led: components.LED
    limelight: components.Limelight
    main_controller: components.XboxController

    def createObjects(self):
        """Create motors and stuff here"""
        # Controller stuff here
        self.main_controller_correct_for_deadband = (
            constants.CONTROLLER_CORRECT_FOR_DEADBAND
        )
        self.main_controller_deadband = constants.CONTROLLER_DEADBAND
        self.main_controller_port = constants.CONTROLLER_PORT
        # LED stuff here
        self.led_length = constants.LED_LENGTH
        self.led_pwm_port = constants.LED_PWM_PORT

    def teleopInit(self):
        """Called when teleop starts; optional"""
        pass

    def teleopPeriodic(self):
        self.main_controller.capture_button_presses()
        left_x, left_y, right_x, right_y = self.main_controller.get_joysticks()

    def disabledInit(self):
        pass

    def disabledPeriodic(self):
        self.led.turn_off()
