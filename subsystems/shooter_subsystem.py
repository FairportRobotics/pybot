from wpilib import TalonFX
from commands2 import SubsystemBase
from constants import MotorIDs, UnnecicaryFancyEnumsForShooting


class ShooterSubsystem(SubsystemBase):
    def __init__(self):
        super().__init__()
        self.m_motor = TalonFX(MotorIDs.kShooterMotor)
        self.m_speed = 0.0
        self.m_mode = None  # type: UnnecicaryFancyEnumsForShooting | None

    def set_speed(self, speed: float):
        """
        Set the speed of the shooter.
        :param speed: Speed between 0 and 1. 0 stops the motor.
        """
        self.m_speed = speed
        if speed != 0:
            self.m_motor.set(speed)
        else:
            self.m_motor.stopMotor()

    def get_speed(self) -> float:
        return self.m_speed

    def get_mode(self) -> UnnecicaryFancyEnumsForShooting | None:
        return self.m_mode

    def set_mode(self, p_mode: UnnecicaryFancyEnumsForShooting):
        self.m_mode = p_mode
