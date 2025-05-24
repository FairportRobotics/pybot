import commands2
from enum import Enum
from robotcontainer import RobotContainer
import time


class ArtilleryPower(Enum):
    FULL = 1
    HALF = 2
    NONE = 3


class ArtilleryCommand(commands2.CommandBase):
    def __init__(self, power: ArtilleryPower):
        super().__init__()
        self._power = power
        self._artillery_subsystem = RobotContainer.ARTILLERYSUBSYSTEM

    def initialize(self):
        if self._power == ArtilleryPower.FULL:
            self._artillery_subsystem.full_power()
        elif self._power == ArtilleryPower.HALF:
            self._artillery_subsystem.half_power()
        elif self._power == ArtilleryPower.NONE:
            self._artillery_subsystem.no_power()

    def execute(self):
        pass

    def end(self, interrupted: bool):
        time.sleep(2)  # Blocking is not ideal; consider alternatives for async delay
        self._artillery_subsystem.no_power()

    def isFinished(self) -> bool:
        return True
