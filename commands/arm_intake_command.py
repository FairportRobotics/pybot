import commands2
from enum import Enum
from robotcontainer import RobotContainer


class ArmIntakeDirection(Enum):
    IN = 1
    OUT = 2


class ArmIntakeCommand(commands2.CommandBase):
    def __init__(self, direction: ArmIntakeDirection):
        super().__init__()
        self._arm_subsystem = RobotContainer.ARM_SUBSYSTEM
        self._direction = direction

    def initialize(self):
        if self._direction == ArmIntakeDirection.IN:
            self._arm_subsystem.intake()
        elif self._direction == ArmIntakeDirection.OUT:
            self._arm_subsystem.outtake()

    def execute(self):
        pass  # Optional, nothing needed here

    def end(self, interrupted: bool):
        self._arm_subsystem.stop()

    def isFinished(self) -> bool:
        return False  # Runs until button is released or command is canceled
