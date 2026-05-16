from phoenix5 import WPI_TalonSRX
import wpilib.drive


class DriveTrain:
    # Injected magic components
    left_leader: WPI_TalonSRX
    left_follower: WPI_TalonSRX
    right_leader: WPI_TalonSRX
    right_follower: WPI_TalonSRX
    # Drivetrain attributes
    speed: float = 0.0
    rotation: float = 0.0

    def setup(self):
        # Get the followers to follow the leader
        self.left_follower.follow(self.left_leader)
        self.right_follower.follow(self.right_leader)
        # Invert the left motors
        self.left_leader.setInverted(True)
        self.left_follower.setInverted(True)
        # Setup the differential drive
        self.drive = wpilib.drive.DifferentialDrive(self.left_leader, self.right_leader)

    def execute(self):
        self.drive.arcadeDrive(self.speed, self.rotation)
