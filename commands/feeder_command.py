from commands2 import CommandBase
from subsystems.feeder_subsystem import FeederSubsystem


class FeederCommand(CommandBase):
    def __init__(self, subsystem: FeederSubsystem, speed: float):
        super().__init__()
        self.m_subsystem = subsystem
        self.m_speed = speed

        # Declare subsystem dependencies
        self.addRequirements(subsystem)

    def execute(self):
        self.m_subsystem.set_motor(self.m_speed)

    def end(self, interrupted: bool):
        self.m_subsystem.set_motor(0)

    def isFinished(self) -> bool:
        return False
