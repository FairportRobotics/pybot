import components
import constants
import magicbot


class DriveForward(magicbot.AutonomousStateMachine):
    MODE_NAME = "Drive Forward"
    DEFAULT = True

    # Set up the drive based on the drivetrain type specified in constants.py
    drivetrain_type = constants.DRIVETRAIN_TYPE.lower()  # Ensure it's lowercase
    if drivetrain_type == "arcade":
        drive: components.ArcadeDrive
    elif drivetrain_type == "tank":
        drive: components.TankDrive
    elif drivetrain_type == "swerve":
        drive: components.SwerveDrive

    @magicbot.timed_state(duration=2.0, next_state="stop", first=True)
    def go(self):
        forward_speed = 0.9
        if self.drivetrain_type == "swerve":
            # For swerve drive, we need to provide x, y, and rotation
            self.drive.go(forward_speed, 0, 0)
        elif self.drivetrain_type == "arcade":
            # For arcade drive, we provide forward speed and rotation
            self.drive.go(forward_speed, 0)
        elif self.drivetrain_type == "tank":
            # For tank drive, we provide left and right speeds
            self.drive.go(forward_speed, forward_speed)

    @magicbot.state()
    def stop(self):
        self.drive.stop()
        self.done()
