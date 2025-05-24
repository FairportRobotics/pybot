import wpilib
import commands2
from ctre import WPI_TalonSRX
from constants import ArmConstants


class ArmSubsystem(commands2.SubsystemBase):
    def __init__(self):
        super().__init__()

        self.arm_solenoid = wpilib.DoubleSolenoid(
            wpilib.PneumaticsModuleType.CTREPCM,
            ArmConstants.ARM_EXTENSION_SOLENOID_ID,
            ArmConstants.ARM_RETRACTION_SOLENOID_ID,
        )

        self.intake_motor = WPI_TalonSRX(14)

    def extend_arm(self):
        self.arm_solenoid.set(wpilib.DoubleSolenoid.Value.kForward)

    def retract_arm(self):
        self.arm_solenoid.set(wpilib.DoubleSolenoid.Value.kReverse)

    def intake(self):
        self.intake_motor.set(1.0)

    def outtake(self):
        self.intake_motor.set(-1.0)

    def stop(self):
        self.intake_motor.set(0.0)
