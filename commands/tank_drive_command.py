import commands2
from robotcontainer import RobotContainer


class TankDriveCommand(commands2.CommandBase):
    def __init__(self):
        super().__init__()
        self._drive_subsystem = RobotContainer.DRIVE_SUBSYSTEM
        self._driver_controller = RobotContainer.driverController

        self.addRequirements(self._drive_subsystem)

    def initialize(self):
        pass

    def execute(self):
        left_speed = self._driver_controller.getLeftY()
        right_speed = self._driver_controller.getRightY()
        self._drive_subsystem.drive(-left_speed, -right_speed)

    def end(self, interrupted: bool):
        pass

    def isFinished(self) -> bool:
        return False
