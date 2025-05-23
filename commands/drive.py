import commands2
import wpilib


class Drive(commands2.CommandBase):
    def __init__(self, drive, controller):
        super().__init__()
        self.drive = drive
        self.controller = controller
        self.addRequirements(drive)

    def execute(self):
        fwd = -self.controller.get_left_y()
        rot = self.controller.get_right_x()
        self.drive.arcade_drive(fwd, rot)
