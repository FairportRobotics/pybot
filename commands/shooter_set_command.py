from commands2 import CommandBase
from subsystems.shooter_subsystem import ShooterSubsystem
from constants import UnnecicaryFancyEnumsForShooting


class ShooterSetCommand(CommandBase):
    def __init__(
        self, subsystem: ShooterSubsystem, mode: UnnecicaryFancyEnumsForShooting
    ):
        super().__init__()
        self.m_subsystem = subsystem
        self.m_mode = mode

        # Declare subsystem dependencies
        self.addRequirements(subsystem)

    def execute(self):
        if (
            self.m_subsystem.get_speed() != 0
            and self.m_subsystem.get_mode() == self.m_mode
        ):
            self.m_subsystem.set_speed(0)
        else:
            if self.m_mode == UnnecicaryFancyEnumsForShooting.slowShoot:
                self.m_subsystem.set_speed(0.5)
            else:
                self.m_subsystem.set_speed(0.7)
            self.m_subsystem.set_mode(self.m_mode)

    def isFinished(self) -> bool:
        return True
