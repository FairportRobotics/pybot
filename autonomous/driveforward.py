import magicbot
import components


class DriveForward(magicbot.AutonomousStateMachine):
    MODE_NAME = "Drive Forward"
    DEFAULT = True
    drivetrain: components.TankDrive

    @magicbot.timed_state(duration=3, first=True)
    def drive_forward(self):
        self.drivetrain.drive(0.5, 0)
