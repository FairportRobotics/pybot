import wpilib
import commands2


class ArtillerySubsystem(commands2.SubsystemBase):
    def __init__(self):
        super().__init__()

        self.shooter_bank_i = wpilib.DoubleSolenoid(
            wpilib.PneumaticsModuleType.CTREPCM, 3, 2
        )
        self.shooter_bank_ii = wpilib.DoubleSolenoid(
            wpilib.PneumaticsModuleType.CTREPCM, 5, 4
        )

        self.shooter_bank_i.set(wpilib.DoubleSolenoid.Value.kReverse)
        self.shooter_bank_ii.set(wpilib.DoubleSolenoid.Value.kReverse)

    def full_power(self):
        self.shooter_bank_i.set(wpilib.DoubleSolenoid.Value.kForward)
        self.shooter_bank_ii.set(wpilib.DoubleSolenoid.Value.kForward)

    def half_power(self):
        self.shooter_bank_i.set(wpilib.DoubleSolenoid.Value.kForward)
        self.shooter_bank_ii.set(wpilib.DoubleSolenoid.Value.kOff)

    def no_power(self):
        self.shooter_bank_i.set(wpilib.DoubleSolenoid.Value.kReverse)
        self.shooter_bank_ii.set(wpilib.DoubleSolenoid.Value.kReverse)
