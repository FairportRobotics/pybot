import commands2
from robotcontainer import RobotContainer
from enum import Enum


class ArmExtentionDirection(Enum):
    EXTEND = 1
    RETRACT = 2


class ArmExtentionCommand(commands2.CommandBase):
    def __init__(self, direction: ArmExtentionDirection):
        super().__init__()
        self._arm_subsystem = RobotContainer.ARM_SUBSYSTEM
        self._direction = direction

    def initialize(self):
        if self._direction == ArmExtentionDirection.EXTEND:
            self._arm_subsystem.extend_arm()
        elif self._direction == ArmExtentionDirection.RETRACT:
            self._arm_subsystem.retract_arm()

    def execute(self):
        pass

    def end(self, interrupted: bool):
        pass

    def isFinished(self) -> bool:
        return True
