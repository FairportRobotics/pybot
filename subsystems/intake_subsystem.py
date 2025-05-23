from wpilib import DoubleSolenoid, PneumaticsModuleType
from phoenix5 import TalonSRX, ControlMode
from commands2 import SubsystemBase
from constants import MotorIDs, SolenoidIDs


class IntakeSubsystem(SubsystemBase):
    def __init__(self):
        super().__init__()
        self.m_solenoid = DoubleSolenoid(
            SolenoidIDs.kHubID,
            PneumaticsModuleType.CTREPCM,
            SolenoidIDs.kFowardIntake,
            SolenoidIDs.kReverseIntake,
        )
        self.m_motor = TalonSRX(MotorIDs.kIntakeMotor)

    def toggle_intake(self):
        # If solenoid is off, set to forward; else toggle between forward and reverse
        if self.m_solenoid.get() == DoubleSolenoid.Value.kOff:
            self.m_solenoid.set(DoubleSolenoid.Value.kForward)
        else:
            self.m_solenoid.toggle()

    def set_motor(self, speed: float):
        """
        Set the intake motor speed.
        :param speed: Value between 0 and 1. 0 stops the motor.
        """
        self.m_motor.set(ControlMode.PercentOutput, speed)
