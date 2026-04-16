import components
from wpilib.drive import DifferentialDrive
from magicbot import will_reset_to


class TankDrive:
    left_motor: components.KrakenMotor
    left_motor_follower: components.KrakenMotor
    right_motor: components.KrakenMotor
    right_motor_follower: components.KrakenMotor
    _throttle = will_reset_to(0.0)
    _rotation = will_reset_to(0.0)

    @property
    def throttle(self):
        return self._throttle

    @throttle.setter
    def throttle(self, value: float):
        self._throttle = value

    @property
    def rotation(self):
        return self._rotation

    @rotation.setter
    def rotation(self, value: float):
        self._rotation = value

    def setup(self):
        self.drive = DifferentialDrive(self.left_motor, self.right_motor)

    def execute(self):
        self.drive.arcadeDrive(self.throttle, self.rotation)
