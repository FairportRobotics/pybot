import constants
from wpilib.drive import DifferentialDrive
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import DifferentialDriveOdometry
import phoenix5
from magicbot import will_reset_to, feedback
# import rev


class TankDrive:
    _speed = will_reset_to(0.0)
    _rotation = will_reset_to(0.0)

    def setup(self) -> None:
        self._pose = Pose2d(0, 0, Rotation2d.fromDegrees(0))
        self._odometry = DifferentialDriveOdometry(
            self._pose.rotation(), 0, 0, self._pose
        )

        self.left_leader = phoenix5.WPI_TalonSRX(constants.CAN_IDs.LEFT_MOTOR)
        self.left_follower = phoenix5.WPI_TalonSRX(
            constants.CAN_IDs.LEFT_FOLLOWER_MOTOR
        )
        self.right_leader = phoenix5.WPI_TalonSRX(constants.CAN_IDs.RIGHT_MOTOR)
        self.right_follower = phoenix5.WPI_TalonSRX(
            constants.CAN_IDs.RIGHT_FOLLOWER_MOTOR
        )
        self.right_leader.setInverted(True)
        self.right_follower.setInverted(True)
        self.left_follower.follow(self.left_leader)
        self.right_follower.follow(self.right_leader)

        # set up differential drive class
        self._drive = DifferentialDrive(self.left_leader, self.right_leader)

    def execute(self) -> None:
        self._odometry.update(
            self._pose.rotation(),
            self.left_leader.getSelectedSensorPosition(),
            self.right_leader.getSelectedSensorPosition(),
        )
        self._pose = self._odometry.getPose()
        self._drive.arcadeDrive(self._speed, self._rotation)

    def drive(self, speed: float, rotation: float) -> None:
        self._speed = speed
        self._rotation = rotation

    def stop(self) -> None:
        self._speed = 0.0
        self._rotation = 0.0
        self._drive.stopMotor()

    @feedback
    def get_speed(self) -> float:
        return self._speed

    @feedback
    def get_rotation(self) -> float:
        return self._rotation

    def pose(self) -> Pose2d:
        return self._pose

    def odometry(self) -> DifferentialDriveOdometry:
        return self._odometry
