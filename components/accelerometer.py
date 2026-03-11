import wpilib
from magicbot import feedback

class RoboRioAccelerometer:
    def setup(self) -> None:
        self.accelerometer = wpilib.BuiltInAccelerometer()
        self.x_accel = 0.0
        self.y_accel = 0.0
        self.z_accel = 0.0

    def execute(self):
        # Get the acceleration values
        self.x_accel = self.accelerometer.getX()
        self.y_accel = self.accelerometer.getY()
        self.z_accel = self.accelerometer.getZ()

    @feedback
    def x(self):
        return self.x_accel
    
    @feedback
    def y(self):
        return self.y_accel
    
    @feedback
    def z(self):
        return self.z_accel