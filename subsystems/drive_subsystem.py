import commands2
from ctre import WPI_TalonSRX
from constants import DriveConstants


class DriveSubsystem(commands2.SubsystemBase):
    def __init__(self):
        super().__init__()

        self.left_main = WPI_TalonSRX(DriveConstants.LEFT_MAIN_TALON_ID)
        self.left_follower = WPI_TalonSRX(DriveConstants.LEFT_FOLLOW_TALON_ID)

        self.right_main = WPI_TalonSRX(DriveConstants.RIGHT_MAIN_TALON_ID)
        self.right_follower = WPI_TalonSRX(DriveConstants.RIGHT_FOLLOW_TALON_ID)

    def periodic(self):
        pass  # Called every robot loop

    def drive(self, left: float, right: float):
        self.left_main.set(left)
        self.left_follower.set(left)

        self.right_main.set(-right)  # Negative if motors are reversed
        self.right_follower.set(-right)
