from commands2 import CommandBase
from subsystems.swerve_drive_subsystem import SwerveDriveSubsystem


class OrientationToggleCommand(CommandBase):
    def __init__(self, subsystem: SwerveDriveSubsystem):
        super().__init__()
        self.m_subsystem = subsystem

        # Declare subsystem dependencies
        self.addRequirements(subsystem)

    def initialize(self):
        # No initialization needed
        pass

    def execute(self):
        self.m_subsystem.toggle_orientation()
        print("yeag")

    def end(self, interrupted: bool):
        # No cleanup needed
        pass

    def isFinished(self) -> bool:
        return True
