from components.led import LED
#from components.controller import XboxController
import constants
from magicbot import MagicRobot, feedback
import wpilib


class MyRobot(MagicRobot):
    #controller: XboxController
    led_strip: LED

    moving = False

    def createObjects(self) -> None:
        """Create motors and stuff here"""
        self.accelerometer = wpilib.BuiltInAccelerometer()
        self.led_strip_length = constants.LED.LENGTH
        self.led_strip_pwm_port = constants.LED.PWM_PORT
        self.controller_port = constants.CONTROLLER_PORT
        
        self.velocity_x = 0.0
        self.distance_x = 0.0
        self.velocity_y = 0.0
        self.distance_y = 0.0
        self.last_time = wpilib.Timer.getFPGATimestamp()

    def teleopPeriodic(self) -> None:
        current_time = wpilib.Timer.getFPGATimestamp()
        dt = current_time - self.last_time
        self.last_time = current_time

        # Get acceleration in Gs, convert to m/s²
        x = self.accelerometer_x()  
        y = self.accelerometer_y()

        self.moving = False

        # Integrate once → velocity
        if x > 0:
            self.velocity_x += x * 9.81 * dt
            self.moving = True
        else:
            self.velocity_x = 0
        
        if y > 0:
            self.velocity_y += y * 9.81 * dt
            self.moving = True
        else:
            self.velocity_y = 0

        # Integrate again → distance
        self.distance_x += self.velocity_x * dt
        self.distance_y += self.velocity_y * dt

        if self.moving:
            self.led_strip.green()
        else:
            self.led_strip.red()

    def disabledPeriodic(self) -> None:
        self.led_strip.turn_off()

    def min_max_filter(self, val, min, max):
        if val < min or val > max:
            return val
        return 0

    def filter2(self, val, mean, sigma):
        # Check if the signal is more than 3 standard deviations from the mean
        if abs(val - mean) > 3 * sigma:
            # It is so don't filter it out
            return val
        # Filter out this value
        return 0
    

    @feedback
    def accelerometer_x(self) -> float:
        #return self.accelerometer.getX()
        return self.min_max_filter(self.accelerometer.getX(), constants.Filter.ACCELEROMETER_X[0], constants.Filter.ACCELEROMETER_X[1])

    @feedback
    def accelerometer_y(self) -> float:
        #return self.accelerometer.getY()
        return self.min_max_filter(self.accelerometer.getY(), constants.Filter.ACCELEROMETER_Y[0], constants.Filter.ACCELEROMETER_Y[1])

    @feedback(key="distance_x")
    def get_distance_x(self):
        return self.distance_x
    
    @feedback(key="distance_y")
    def get_distance_y(self):
        return self.distance_y