import constants
import components
from wpilib.drive import DifferentialDrive
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import DifferentialDriveOdometry
from wpimath.units import inchesToMeters
import phoenix5
import navx
from magicbot import will_reset_to, feedback
# from phoenix5.sensors import WPI_PigeonIMU
# import rev


class TankDrive:
    _speed = will_reset_to(0.0)
    _rotation = will_reset_to(0.0)
    motors: dict[str, phoenix5.WPI_TalonSRX]
    gyro: navx.AHRS

    def setup(self) -> None:
        self._pose = Pose2d(0, 0, Rotation2d.fromDegrees(0))
        self._odometry = DifferentialDriveOdometry(
            self._pose.rotation(), 0, 0, self._pose
        )

        self.left_motor = self.motors["left_motor"]
        # We are going to invert the right motors
        self.right_motor = self.motors["right_motor"]
        self.right_motor.setInverted(True)
        # Check if there are follower motors, and if so, set them up
        if "right_follower" in self.motors:
            self.right_follower = self.motors["right_follower"]
            self.right_follower.setInverted(True)
            self.right_follower.follow(self.right_motor)
        if "left_follower" in self.motors:
            self.left_follower = self.motors["left_follower"]
            self.left_follower.follow(self.left_motor)

        # set up differential drive class
        self._drive = DifferentialDrive(self.left_motor, self.right_motor)

    def reset(self) -> None:
        self._pose = Pose2d(0, 0, Rotation2d.fromDegrees(0))
        self.left_motor.setSelectedSensorPosition(0)
        self.right_motor.setSelectedSensorPosition(0)

    def execute(self) -> None:
        gyro_angle = Rotation2d.fromDegrees(-self.gyro.getAngle())
        self._odometry.update(
            gyro_angle,
            self.left_motor.getSelectedSensorPosition(),
            self.right_motor.getSelectedSensorPosition(),
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

    def _sensor_units_per_100ms_to_mps(self, sensor_velocity: float) -> float:
        """Convert TalonSRX native velocity units to meters per second.

        TalonSRX reports velocity in sensor units per 100ms.
        """
        rotations_per_100ms = (
            sensor_velocity / constants.Robot.ENCODER_TICKS_PER_ROTATION
        )
        rotations_per_second = rotations_per_100ms * 10
        wheel_rotations_per_second = rotations_per_second / constants.Robot.GEAR_RATIO
        return wheel_rotations_per_second * inchesToMeters(
            constants.Robot.WHEEL_RADIUS_IN_INCHES
        )

    @feedback
    def get_left_velocity(self) -> float:
        """Left drivetrain velocity in meters per second."""
        return self._sensor_units_per_100ms_to_mps(
            self.left_motor.getSelectedSensorVelocity()
        )

    @feedback
    def get_right_velocity(self) -> float:
        """Right drivetrain velocity in meters per second."""
        return self._sensor_units_per_100ms_to_mps(
            self.right_motor.getSelectedSensorVelocity()
        )

    @feedback
    def get_average_velocity(self) -> float:
        """Average drivetrain velocity in meters per second (positive = forward)."""
        return (self.get_left_velocity() + self.get_right_velocity()) / 2.0
