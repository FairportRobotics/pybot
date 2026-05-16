from magicbot import feedback
from phoenix5 import WPI_TalonSRX


class Roller:
    # Injected magic components
    motor: WPI_TalonSRX
    # Roller attributes
    speed: float = 0.0

    def execute(self):
        self.motor.set(self.speed)

    @feedback(key="speed")
    def get_speed(self):
        return self.speed
