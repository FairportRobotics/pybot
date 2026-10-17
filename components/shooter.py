from magicbot import will_reset_to
import components


class Shooter:
    # MagicBot injects these by name from robot.py createObjects()
    shooter_motor: components.KrakenMotor
    feeder_motor: components.NeoMotor

    target_speed = will_reset_to(0.0)

    def shoot(self, speed: float) -> None:
        self.target_speed = speed

    def execute(self) -> None:
        self.shooter_motor.set_output(self.target_speed)
        self.feeder_motor.set_output(self.target_speed * 0.5)
