"""SwerveDrive component.

Top-level MagicBot component that owns the four ``SwerveModule`` instances,
the Pigeon 2 gyro, and all kinematics / odometry objects.

Responsibilities
----------------
* Accept a chassis velocity command (``vx``, ``vy``, ``omega``) each loop.
* Convert the command to per-module ``SwerveModuleState`` objects via
  ``SwerveDrive4Kinematics``.
* Delegate the per-module states to each ``SwerveModule``.
* Maintain field-relative pose estimation via ``SwerveDrive4PoseEstimator``,
  which fuses wheel odometry with vision measurements from the Limelight
  cameras when ``add_vision_measurement()`` is called.
* Support toggling between robot-relative and field-relative driving.
"""

from __future__ import annotations

import math

import wpilib
from wpimath.controller import PIDController

if wpilib.RobotBase.isSimulation():
    from components.sim_devices import SimPigeon2
else:
    import phoenix6.hardware
from magicbot import feedback
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import (
    ChassisSpeeds,
    SwerveDrive4Kinematics,
    SwerveModuleState,
)

import constants
from components.swerve_module import SwerveModule


class SwerveDrive:
    """MagicBot component — four-wheel swerve drivetrain with odometry.

    MagicBot instantiates this class and calls ``execute()`` every robot loop.
    The ``robot.py`` driver station code calls ``drive()`` to set the desired
    chassis velocity.
    """

    def __init__(self) -> None:
        # --- Gyro ---
        # Use a lightweight stub in simulation to avoid the Phoenix 6
        # standalone package crashing under pyfrc HAL-sim.
        if wpilib.RobotBase.isSimulation():
            self._gyro = SimPigeon2()
        else:
            self._gyro = phoenix6.hardware.Pigeon2(constants.PIGEON_ID, constants.CANIVORE_BUS)

        # --- Swerve modules (FL, FR, BL, BR) ---
        self._modules: tuple[SwerveModule, SwerveModule, SwerveModule, SwerveModule] = (
            SwerveModule(  # Front-Left
                constants.DRIVE_MOTOR_IDS[0],
                constants.STEER_MOTOR_IDS[0],
                constants.CANCODER_IDS[0],
                constants.CANCODER_OFFSETS_RAD[0],
            ),
            SwerveModule(  # Front-Right
                constants.DRIVE_MOTOR_IDS[1],
                constants.STEER_MOTOR_IDS[1],
                constants.CANCODER_IDS[1],
                constants.CANCODER_OFFSETS_RAD[1],
                drive_inverted=True,
            ),
            SwerveModule(  # Back-Left
                constants.DRIVE_MOTOR_IDS[2],
                constants.STEER_MOTOR_IDS[2],
                constants.CANCODER_IDS[2],
                constants.CANCODER_OFFSETS_RAD[2],
            ),
            SwerveModule(  # Back-Right
                constants.DRIVE_MOTOR_IDS[3],
                constants.STEER_MOTOR_IDS[3],
                constants.CANCODER_IDS[3],
                constants.CANCODER_OFFSETS_RAD[3],
                drive_inverted=True,
            ),
        )

        # --- Kinematics ---
        # Module positions relative to robot centre (x = forward, y = left).
        half_wb = constants.WHEELBASE_M / 2.0
        half_tw = constants.TRACK_WIDTH_M / 2.0
        self._kinematics = SwerveDrive4Kinematics(
            Translation2d(half_wb, half_tw),  # Front-Left
            Translation2d(half_wb, -half_tw),  # Front-Right
            Translation2d(-half_wb, half_tw),  # Back-Left
            Translation2d(-half_wb, -half_tw),  # Back-Right
        )

        # --- Pose Estimator ---
        # SwerveDrive4PoseEstimator extends plain odometry by accepting
        # vision measurements with configurable noise standard deviations.
        # State std devs: [x_m, y_m, heading_rad] — wheel odometry noise model.
        self._pose_estimator = SwerveDrive4PoseEstimator(
            self._kinematics,
            self._get_rotation2d(),
            self._get_module_positions(),
            Pose2d(),
        )

        # --- Field2d widget ---
        # Publishes the robot's estimated pose to SmartDashboard so Elastic /
        # Shuffleboard can overlay it on a field image.  Also used by
        # PathPlanner to display the active trajectory during auto.
        self._field = wpilib.Field2d()
        wpilib.SmartDashboard.putData("Field", self._field)

        # --- Commanded chassis speeds (set each loop by drive()) ---
        self._chassis_speeds = ChassisSpeeds(0.0, 0.0, 0.0)

        # --- Driving mode ---
        self._field_relative: bool = False  # Robot-relative by default

        # --- Heading hold ---
        # When the driver releases the rotation stick (omega ≈ 0), we lock
        # onto the current heading and use a PID controller to correct drift.
        # This keeps the robot driving in a straight line without the driver
        # having to constantly micro-adjust the rotation stick.
        self._heading_hold_pid = PIDController(
            constants.HEADING_KP,
            constants.HEADING_KI,
            constants.HEADING_KD,
        )
        # PID operates on a circular quantity — tell it that 2π = 0.
        self._heading_hold_pid.enableContinuousInput(-math.pi, math.pi)
        # Target heading (radians). Updated whenever the driver steers.
        self._locked_heading_rad: float = 0.0

        # --- SysId override flag ---
        # When True, execute() skips normal closed-loop drive so that
        # DrivetrainSysId can apply open-loop voltages directly.
        self._sysid_active: bool = False

    # ------------------------------------------------------------------
    # Public API — called by robot.py each teleop loop
    # ------------------------------------------------------------------

    def drive(
        self,
        vx: float,
        vy: float,
        omega: float,
        field_relative: bool | None = None,
    ) -> None:
        """Command the drivetrain with a chassis velocity.

        Parameters
        ----------
        vx:
            Forward velocity in metres/second (+x = towards opposing alliance).
        vy:
            Lateral velocity in metres/second (+y = left).
        omega:
            Rotation rate in radians/second (counter-clockwise positive).
        field_relative:
            Override the current driving mode for this call only.  If
            ``None``, the mode set by ``set_field_relative()`` is used.
        """
        use_field_relative = self._field_relative if field_relative is None else field_relative

        # --- Heading hold ---
        # If the driver is actively rotating (|omega| above threshold), follow
        # their input and update the locked heading for when they release.
        # If omega is near zero, substitute a PID correction to maintain the
        # last locked heading instead — this keeps the robot straight.
        if abs(omega) >= constants.HEADING_HOLD_OMEGA_THRESHOLD_RAD_S:
            # Driver is steering — use their input and save the current heading.
            actual_omega = omega
            self._locked_heading_rad = self.get_heading_rad()
            self._heading_hold_pid.reset()
        else:
            # Driver released the stick — correct any drift back to locked heading.
            actual_omega = self._heading_hold_pid.calculate(
                self.get_heading_rad(), self._locked_heading_rad
            )

        if use_field_relative:
            self._chassis_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                vx, vy, actual_omega, self._get_rotation2d()
            )
        else:
            self._chassis_speeds = ChassisSpeeds(vx, vy, actual_omega)

    def stop(self) -> None:
        """Command all modules to stop (zero velocity, hold current angle)."""
        self._chassis_speeds = ChassisSpeeds(0.0, 0.0, 0.0)

    def get_chassis_speeds(self) -> ChassisSpeeds:
        """Return the measured robot-relative chassis speeds from wheel encoders.

        PathPlanner calls this every loop to know how fast the robot is actually
        moving (not just what we commanded).  The kinematics object converts
        the four individual module velocities/angles into a single
        ``ChassisSpeeds`` (vx, vy, omega) in the robot frame.
        """
        return self._kinematics.toChassisSpeeds(self._get_module_states())

    def drive_chassis_speeds(self, speeds: ChassisSpeeds) -> None:
        """Accept a robot-relative ``ChassisSpeeds`` command from PathPlanner.

        PathPlanner calls this output function each loop with the speeds
        needed to follow the current path.  We store the value and let
        ``execute()`` distribute it to the four modules — exactly the same
        path as a normal ``drive()`` call, so all existing safety logic
        (desaturation, SysId bypass) still applies.

        Parameters
        ----------
        speeds:
            Robot-relative target chassis speeds computed by the PathPlanner
            holonomic controller.  Units: m/s for vx/vy, rad/s for omega.
        """
        self._chassis_speeds = speeds

    def set_field_relative(self, enabled: bool) -> None:
        """Enable or disable field-relative driving mode.

        Parameters
        ----------
        enabled:
            ``True`` → field-relative; ``False`` → robot-relative.
        """
        self._field_relative = enabled

    def toggle_field_relative(self) -> None:
        """Flip between field-relative and robot-relative driving."""
        self._field_relative = not self._field_relative

    @property
    def is_field_relative(self) -> bool:
        """``True`` when the drivetrain is in field-relative mode."""
        return self._field_relative

    def reset_gyro(self) -> None:
        """Zero the gyro, treating the current heading as 0°."""
        self._gyro.set_yaw(0.0)
        self._locked_heading_rad = 0.0
        self._heading_hold_pid.reset()

    def get_heading_rad(self) -> float:
        """Return the current robot heading in radians (CCW positive)."""
        return math.radians(self._gyro.get_yaw().value)

    def get_pose(self) -> Pose2d:
        """Return the current estimated field pose from the pose estimator."""
        return self._pose_estimator.getEstimatedPosition()

    def reset_pose(self, pose: Pose2d) -> None:
        """Reset the pose estimator to a known pose (e.g., at match start)."""
        self._pose_estimator.resetPosition(
            self._get_rotation2d(),
            self._get_module_positions(),
            pose,
        )

    def add_vision_measurement(
        self,
        vision_pose: Pose2d,
        timestamp_s: float,
        std_devs: tuple[float, float, float],
    ) -> None:
        """Fuse a vision-derived pose estimate into the pose estimator.

        Called by the ``Limelight`` component each loop when a high-quality
        AprilTag estimate is available.

        Parameters
        ----------
        vision_pose:
            Robot pose estimated from AprilTag vision in field coordinates.
        timestamp_s:
            FPGA timestamp (seconds) when the camera frame was captured,
            compensated for pipeline and capture latency.
        std_devs:
            Measurement noise standard deviations ``(x_m, y_m, heading_rad)``.
            Larger values = trust vision less relative to wheel odometry.
            Use ``float("inf")`` for heading to ignore the vision heading.
        """
        self._pose_estimator.addVisionMeasurement(
            vision_pose,
            timestamp_s,
        )

    # ------------------------------------------------------------------
    # Telemetry — published to NetworkTables via MagicBot @feedback
    # Each method is called every loop and its return value is written
    # to SmartDashboard under "components/swerve_drive/<method_name>".
    # ------------------------------------------------------------------

    @feedback
    def heading_degrees(self) -> float:
        """Current robot heading in degrees (CCW positive, 0 = forward)."""
        return math.degrees(self.get_heading_rad())

    @feedback
    def pose_x(self) -> float:
        """Odometry X position on the field in metres."""
        return self.get_pose().X()

    @feedback
    def pose_y(self) -> float:
        """Odometry Y position on the field in metres."""
        return self.get_pose().Y()

    @feedback
    def pose_rotation_degrees(self) -> float:
        """Odometry heading estimate in degrees."""
        return self.get_pose().rotation().degrees()

    @feedback
    def velocity_x_mps(self) -> float:
        """Commanded forward velocity in metres/second."""
        return self._chassis_speeds.vx

    @feedback
    def velocity_y_mps(self) -> float:
        """Commanded lateral velocity in metres/second."""
        return self._chassis_speeds.vy

    @feedback
    def angular_velocity_dps(self) -> float:
        """Commanded rotation rate in degrees/second."""
        return math.degrees(self._chassis_speeds.omega)

    @feedback
    def field_relative_enabled(self) -> bool:
        """``True`` when field-relative driving mode is active."""
        return self._field_relative

    # ------------------------------------------------------------------
    # System identification helpers
    # ------------------------------------------------------------------

    def set_open_loop_voltage(self, volts: float) -> None:
        """Apply the same open-loop voltage to all four drive motors.

        Called by ``DrivetrainSysId`` during characterisation routines.
        Sets ``_sysid_active = True`` so ``execute()`` skips normal drive.

        Parameters
        ----------
        volts:
            Voltage to apply.  Positive = forward.  Clamped to ±SYSID_MAX_VOLTAGE_V.
        """
        volts = max(-constants.SYSID_MAX_VOLTAGE_V, min(constants.SYSID_MAX_VOLTAGE_V, volts))
        self._sysid_active = True
        for module in self._modules:
            module.set_drive_voltage(volts)

    def stop_sysid(self) -> None:
        """End an open-loop SysId run and return to closed-loop drive."""
        self._sysid_active = False
        for module in self._modules:
            module.stop()

    @feedback
    def get_avg_drive_position_m(self) -> float:
        """Average drive position across all four modules (metres)."""
        return sum(m.get_drive_position_m() for m in self._modules) / len(self._modules)

    @feedback
    def get_avg_drive_velocity_mps(self) -> float:
        """Average drive velocity across all four modules (m/s)."""
        return sum(m.get_drive_velocity_mps() for m in self._modules) / len(self._modules)

    # ------------------------------------------------------------------
    # MagicBot interface
    # ------------------------------------------------------------------

    def execute(self) -> None:
        """Apply the commanded chassis speeds to all four modules.

        Called automatically by MagicBot every robot loop.
        When ``_sysid_active`` is ``True``, skips normal closed-loop drive
        so that ``DrivetrainSysId`` can hold open-loop voltage control.
        """
        if not self._sysid_active:
            module_states: tuple[
                SwerveModuleState,
                SwerveModuleState,
                SwerveModuleState,
                SwerveModuleState,
            ] = self._kinematics.toSwerveModuleStates(self._chassis_speeds)

            # Desaturate so no module exceeds the physical speed limit.
            SwerveDrive4Kinematics.desaturateWheelSpeeds(
                module_states, constants.TELEOP_MAX_SPEED_MPS
            )

            for module, state in zip(self._modules, module_states, strict=False):
                module.set_desired_state(state)
                module.apply_state()

        # Always update pose estimator with latest wheel and gyro readings.
        self._pose_estimator.update(
            self._get_rotation2d(),
            self._get_module_positions(),
        )

        # Push the latest estimated pose to the Field2d widget so it appears
        # on the Elastic / Shuffleboard field overlay in real time.
        self._field.setRobotPose(self.get_pose())

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _get_rotation2d(self) -> Rotation2d:
        """Read the gyro yaw and wrap it into a ``Rotation2d``."""
        return Rotation2d(self.get_heading_rad())

    def _get_module_positions(
        self,
    ) -> tuple:
        """Return a tuple of ``SwerveModulePosition`` for all four modules."""
        return tuple(m.get_position() for m in self._modules)

    def _get_module_states(self) -> tuple:
        """Return a tuple of current measured ``SwerveModuleState`` for all four modules.

        Used by ``get_chassis_speeds()`` to compute actual robot velocity from
        encoder readings rather than from the last commanded value.
        """
        return tuple(m.get_state() for m in self._modules)
