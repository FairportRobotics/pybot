"""
swerve_drive.py - MagicBot Component for a full 4-module swerve drivetrain.

Depends on four SwerveModule components injected by robot.py.
Uses a NavX (or Pigeon 2) for field-relative driving and WPILib kinematics.
"""

import constants
import math
import wpilib
from magicbot import tunable
from wpimath.geometry import Rotation2d, Pose2d, Translation2d
from wpimath.kinematics import (
    SwerveDrive4Kinematics,
    SwerveDrive4Odometry,
    ChassisSpeeds,
)
from components.swervemodule import SwerveModule
from components.gyro import NavX2


class SwerveDrive:
    """
    MagicBot component that coordinates four SwerveModule components
    to provide field-relative or robot-relative swerve driving.

    Inject the four SwerveModule components plus the gyro in robot.py.

    Example robot.py injection:
        # Modules (positions relative to robot centre, in metres)
        self.front_left: SwerveModule
        self.front_right: SwerveModule
        self.rear_left: SwerveModule
        self.rear_right: SwerveModule

        self.swerve_drive: SwerveDrive

        # CAN IDs  (drive, steer, cancoder, offset_rotations)
        self.front_left_drive_id   = 1;  self.front_left_steer_id   = 2
        self.front_left_cancoder_id = 3; self.front_left_cancoder_offset = 0.0

        self.front_right_drive_id   = 4; self.front_right_steer_id   = 5
        self.front_right_cancoder_id = 6; self.front_right_cancoder_offset = 0.0

        self.rear_left_drive_id   = 7;   self.rear_left_steer_id   = 8
        self.rear_left_cancoder_id = 9;  self.rear_left_cancoder_offset = 0.0

        self.rear_right_drive_id   = 10; self.rear_right_steer_id   = 11
        self.rear_right_cancoder_id = 12; self.rear_right_cancoder_offset = 0.0

        self.gyro_id = 13   # Pigeon 2 CAN ID
    """

    # --- Injected sub-components ---
    front_left: SwerveModule
    front_right: SwerveModule
    rear_left: SwerveModule
    rear_right: SwerveModule
    gyro: NavX2

    # --- Tunables ---
    max_speed_mps = tunable(4.5)  # m/s
    max_angular_speed_rps = tunable(2 * math.pi)  # rad/s

    def setup(self):
        """Called by MagicBot after injection."""
        # Module translation vectors (FL, FR, RL, RR)
        half_x = constants.WHEEL_BASE / 2.0
        half_y = constants.TRACK_WIDTH / 2.0

        self.kinematics = SwerveDrive4Kinematics(
            Translation2d(half_x, half_y),  # front left
            Translation2d(half_x, -half_y),  # front right
            Translation2d(-half_x, half_y),  # rear left
            Translation2d(-half_x, -half_y),  # rear right
        )

        self.odometry = SwerveDrive4Odometry(
            self.kinematics,
            self.get_heading(),
            (
                self.front_left.get_position(),
                self.front_right.get_position(),
                self.rear_left.get_position(),
                self.rear_right.get_position(),
            ),
            Pose2d(),
        )

        # Internal chassis speeds (set by drive methods)
        self.chassis_speeds = ChassisSpeeds(0.0, 0.0, 0.0)
        self.field_relative = True

        # Publish to SmartDashboard
        self.field = wpilib.Field2d()
        wpilib.SmartDashboard.putData("Field", self.field)

    # ------------------------------------------------------------------
    # Public driving API  (called from teleop/autonomous components)
    # ------------------------------------------------------------------

    def drive(
        self,
        x_speed: float,
        y_speed: float,
        rotation: float,
        field_relative: bool = True,
    ):
        """
        Drive the robot.

        Args:
            x_speed:   Forward speed, m/s  (positive = forward)
            y_speed:   Left speed, m/s      (positive = left)
            rotation:  Angular rate, rad/s  (positive = counter-clockwise)
            field_relative: If True, x/y are relative to the field.
        """
        self.field_relative = field_relative
        if self.field_relative:
            self.chassis_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                x_speed, y_speed, rotation, self.get_heading()
            )
        else:
            self.chassis_speeds = ChassisSpeeds(x_speed, y_speed, rotation)

    def stop(self):
        """Command all modules to stop and hold position."""
        self.chassis_speeds = ChassisSpeeds(0.0, 0.0, 0.0)

    def reset_pose(self, pose: Pose2d = Pose2d()):
        """Reset odometry to a known pose (e.g. from vision or auto)."""
        self.odometry.resetPosition(
            self.get_heading(),
            (
                self.front_left.get_position(),
                self.front_right.get_position(),
                self.rear_left.get_position(),
                self.rear_right.get_position(),
            ),
            pose,
        )

    def get_pose(self) -> Pose2d:
        """Return the estimated field-relative robot pose."""
        return self.odometry.getPose()

    def get_heading(self) -> Rotation2d:
        return self.gyro.heading()

    def zero_heading(self):
        self.gyro.reset()

    # ------------------------------------------------------------------
    # MagicBot execute
    # ------------------------------------------------------------------

    def execute(self):
        # Discretise chassis speeds to reduce skew during rotation
        speeds = ChassisSpeeds.discretize(
            self.chassis_speeds, wpilib.Timer.getFPGATimestamp()
        )

        # Compute individual module states
        module_states = self.kinematics.toSwerveModuleStates(speeds)

        # Desaturate: scale all modules so none exceeds max_speed
        module_states = SwerveDrive4Kinematics.desaturateWheelSpeeds(
            module_states, self.max_speed_mps
        )

        fl, fr, rl, rr = module_states
        self.front_left.set_desired_state(fl)
        self.front_right.set_desired_state(fr)
        self.rear_left.set_desired_state(rl)
        self.rear_right.set_desired_state(rr)

        # Update odometry
        self.odometry.update(
            self.get_heading(),
            (
                self.front_left.get_position(),
                self.front_right.get_position(),
                self.rear_left.get_position(),
                self.rear_right.get_position(),
            ),
        )

        # SmartDashboard telemetry
        pose = self.get_pose()
        self.field.setRobotPose(pose)
        wpilib.SmartDashboard.putNumber("Heading (deg)", self.get_heading().degrees())
        wpilib.SmartDashboard.putNumber("Pose X (m)", pose.x)
        wpilib.SmartDashboard.putNumber("Pose Y (m)", pose.y)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _get_gyro_rotation(self) -> Rotation2d:
        """Return gyro yaw as a Rotation2d (negated for CCW-positive convention)."""
        return Rotation2d.fromDegrees(-self._gyro.get_yaw().value)
