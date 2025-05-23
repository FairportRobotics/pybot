from wpilib import TalonSRX
from phoenix5 import ControlMode
from commands2 import SubsystemBase
from constants import MotorIDs


class AgitatorSubsystem(SubsystemBase):
    def __init__(self):
        super().__init__()
        self.m_motor = TalonSRX(MotorIDs.kAgitatorMotor)

    def set_motor(self, speed: float):
        """
        Sets the Agitator motor speed.
        :param speed: A value between 0 and 1. 0 will stop the motor.
        """
        self.m_motor.set(ControlMode.PercentOutput, speed)
