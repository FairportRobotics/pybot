from wpilib import SmartDashboard
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import ChassisSpeeds, SwerveDriveKinematics
from commands2 import SubsystemBase
from navx import AHRS
from constants import DriveConstants
from subsystems.swerve_module import SwerveModule


class SwerveDriveSubsystem(SubsystemBase):
    def __init__(self, gyro: AHRS, module_positions: list[Translation2d]):
        super().__init__()

        # Gyro sensor (navX)
        self.gyro = gyro
        self.gyro.reset()

        # Swerve module positions (Translation2d)
        self.module_positions = module_positions

        # Swerve modules initialization: You should pass motors, encoders, etc.
        self.modules = [
            SwerveModule(module_number=0, module_position=module_positions[0]),
            SwerveModule(module_number=1, module_position=module_positions[1]),
            SwerveModule(module_number=2, module_position=module_positions[2]),
            SwerveModule(module_number=3, module_position=module_positions[3]),
        ]

        # Kinematics
        self.kinematics = SwerveDriveKinematics(*module_positions)

        # Pose of the robot on the field
        self.pose = Pose2d()

        # Field-relative drive mode flag
        self.field_relative = True

        # Cached orientation toggle
        self.orientation_flipped = False

    def toggle_orientation(self):
        self.orientation_flipped = not self.orientation_flipped
        print(f"Orientation flipped: {self.orientation_flipped}")

    def get_heading(self) -> Rotation2d:
        # navX gives yaw in degrees, convert to Rotation2d
        angle = self.gyro.getYaw()
        if self.orientation_flipped:
            angle += 180.0
        return Rotation2d.fromDegrees(angle)

    def get_pose(self) -> Pose2d:
        return self.pose

    def reset_pose(self, pose: Pose2d):
        self.pose = pose
        self.gyro.reset()

    def drive(
        self, x_speed: float, y_speed: float, rot: float, field_relative: bool = True
    ):
        """
        Drive the robot using specified speeds and rotation.

        :param x_speed: Speed in meters per second in x direction
        :param y_speed: Speed in meters per second in y direction
        :param rot: Rotational speed in radians per second
        :param field_relative: Use field-relative controls or robot-relative
        """
        if field_relative:
            # Use current heading to convert field relative speeds to robot relative
            heading = self.get_heading()
            chassis_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                x_speed, y_speed, rot, heading
            )
        else:
            chassis_speeds = ChassisSpeeds(x_speed, y_speed, rot)

        # Calculate individual module states
        module_states = self.kinematics.toSwerveModuleStates(chassis_speeds)

        # Normalize wheel speeds if any exceed max speed
        SwerveDriveKinematics.normalizeWheelSpeeds(
            module_states, DriveConstants.kMaxSpeedMetersPerSecond
        )

        # Set each module to desired state
        for module, state in zip(self.modules, module_states):
            module.set_desired_state(state)

    def periodic(self):
        """
        Called periodically to update pose and send data to dashboard.
        """
        # Update pose estimation here using encoders + gyro data (odometry)
        # This requires your swerve modules to provide current state/position
        # For simplicity, we'll just leave pose as is.

        # Display data on SmartDashboard
        SmartDashboard.putNumber("Robot Heading", self.get_heading().degrees())
        SmartDashboard.putBoolean("Field Relative", self.field_relative)

        # For debugging: show each module state
        for i, module in enumerate(self.modules):
            state = module.get_state()
            SmartDashboard.putNumber(f"Module {i} Angle", state.angle.degrees())
            SmartDashboard.putNumber(f"Module {i} Speed", state.speed)
