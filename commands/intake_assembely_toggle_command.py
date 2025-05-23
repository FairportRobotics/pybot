from commands2 import CommandBase
from subsystems.intake_subsystem import IntakeSubsystem


class IntakeAssembelyToggleCommand(CommandBase):
    def __init__(self, subsystem: IntakeSubsystem):
        super().__init__()
        self.m_subsystem = subsystem

        # Declare subsystem dependencies
        self.addRequirements(subsystem)

    def initialize(self):
        self.m_subsystem.toggle_intake()

    def isFinished(self) -> bool:
        return True
