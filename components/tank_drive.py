"""TankDrive component.

A drop-in MagicBot drivetrain component for a differential (tank) drive
robot.  Although this project uses swerve drive, this component is
provided as a reference implementation and for use on future robots.

Responsibilities
----------------
* Accept a chassis velocity command (``vx``, ``omega``) each loop via
  ``drive()`` or ``drive_arcade()`` / ``drive_tank()``.
* Convert the command to left/right wheel speeds via
  ``DifferentialDriveKinematics``.
* Apply those speeds to the left and right ``TalonFX`` motor pairs.
* Maintain field-relative pose estimation via
  ``DifferentialDrivePoseEstimator``, which fuses wheel odometry with
  vision measurements from Limelight cameras when
  ``add_vision_measurement()`` is called.

AdvantageScope / NT struct topics (all under ``/TankDrive/``)
--------------------------------------------------------------
* ``/TankDrive/Pose``            — robot pose as a ``Pose2d`` struct
* ``/TankDrive/CommandedSpeeds`` — chassis speeds from ``drive()`` (struct)
* ``/TankDrive/MeasuredSpeeds``  — chassis speeds back-computed from encoders
* ``/TankDrive/WheelSpeeds``     — left/right wheel speeds (struct)

NetworkTables ``@feedback`` keys
---------------------------------
*(all under ``SmartDashboard/components/tank_drive/``)*

* ``heading_degrees``       — gyro yaw (degrees)
* ``pose_x``                — odometry X (m)
* ``pose_y``                — odometry Y (m)
* ``pose_rotation_degrees`` — odometry heading (degrees)
* ``left_velocity_mps``     — measured left side speed (m/s)
* ``right_velocity_mps``    — measured right side speed (m/s)
* ``velocity_x_mps``        — commanded forward speed (m/s)
* ``angular_velocity_dps``  — commanded rotation rate (deg/s)

Field layout
------------
::

    Left motors  ← ─────── robot ──────── →  Right motors
     (positive = forward)          (positive = forward)

    Positive vx  = forward
    Positive omega = counter-clockwise (left turn)

Wiring assumptions (adjust CAN IDs in ``constants.py``)
--------------------------------------------------------
* Two motors per side — leader + follower — each a ``TalonFX`` on the
  CANivore bus.  The follower is configured to follow the leader.
* A Pigeon 2 gyro on the CANivore bus (shared with the swerve gyro ID).
* Encoders are built into the TalonFX (Falcon 500 / Kraken X60).
"""

from __future__ import annotations

import math

import ntcore
import wpilib
from magicbot import feedback
from wpimath.estimator import DifferentialDrivePoseEstimator
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import (
    ChassisSpeeds,
    DifferentialDriveKinematics,
    DifferentialDriveWheelSpeeds,
)

import constants

if wpilib.RobotBase.isSimulation():
    from components.sim_devices import SimPigeon2, SimTalonFX
else:
    import phoenix6.configs
    import phoenix6.controls
    import phoenix6.hardware
    from phoenix6.signals.spn_enums import MotorAlignmentValue


# ---------------------------------------------------------------------------
# Constants — these would normally live in constants.py for a real robot.
# They are defined inline here so tank_drive.py is self-contained as a
# reference implementation.
# ---------------------------------------------------------------------------

#: CAN IDs for the left drive motors (leader, follower).
_TANK_LEFT_IDS: tuple[int, int] = (20, 21)
#: CAN IDs for the right drive motors (leader, follower).
_TANK_RIGHT_IDS: tuple[int, int] = (22, 23)
#: Gear ratio between motor shaft and wheel (motor rotations per wheel rotation).
_TANK_GEAR_RATIO: float = 10.71  # typical FRC 3-CIM gearbox ratio
#: Wheel radius in metres (6-inch wheel = 0.0762 m radius).
_TANK_WHEEL_RADIUS_M: float = 0.0762
#: Track width — lateral distance between left and right wheel contact patches (m).
_TANK_TRACK_WIDTH_M: float = constants.TRACK_WIDTH_M
#: Maximum drive speed in m/s — used for desaturation.
_TANK_MAX_SPEED_MPS: float = 4.0
#: Maximum stator current per drive motor (A) — protects gearboxes.
_TANK_STATOR_LIMIT_A: float = 60.0
#: Wheel circumference — pre-computed to avoid repeated math at runtime.
_TANK_WHEEL_CIRCUMFERENCE_M: float = _TANK_WHEEL_RADIUS_M * 2.0 * math.pi


class TankDrive:
    """MagicBot component — two-side differential (tank) drivetrain with odometry.

    MagicBot instantiates this class and calls ``execute()`` every robot loop.
    Call ``drive()`` (or its variants) to command the robot.

    To use this component, declare it in ``robot.py``::

        tank_drive: TankDrive

    and remove (or comment out) the ``swerve_drive`` declaration.
    """

    def __init__(self) -> None:
        # --- Gyro ---
        if wpilib.RobotBase.isSimulation():
            self._gyro = SimPigeon2()
        else:
            self._gyro = phoenix6.hardware.Pigeon2(constants.PIGEON_ID, constants.CANIVORE_BUS)

        # --- Drive motors ---
        # Each side has a leader (closed-loop velocity control) and a
        # follower (mirrors the leader via Phoenix 6 follower control).
        if wpilib.RobotBase.isSimulation():
            self._left_leader: SimTalonFX | phoenix6.hardware.TalonFX = SimTalonFX()
            self._left_follower: SimTalonFX | phoenix6.hardware.TalonFX = SimTalonFX()
            self._right_leader: SimTalonFX | phoenix6.hardware.TalonFX = SimTalonFX()
            self._right_follower: SimTalonFX | phoenix6.hardware.TalonFX = SimTalonFX()
        else:
            self._left_leader = phoenix6.hardware.TalonFX(_TANK_LEFT_IDS[0], constants.CANIVORE_BUS)
            self._left_follower = phoenix6.hardware.TalonFX(
                _TANK_LEFT_IDS[1], constants.CANIVORE_BUS
            )
            self._right_leader = phoenix6.hardware.TalonFX(
                _TANK_RIGHT_IDS[0], constants.CANIVORE_BUS
            )
            self._right_follower = phoenix6.hardware.TalonFX(
                _TANK_RIGHT_IDS[1], constants.CANIVORE_BUS
            )
            self._configure_motors()

        # --- Kinematics ---
        # Converts ChassisSpeeds (vx, omega) ↔ left/right wheel speeds.
        self._kinematics = DifferentialDriveKinematics(_TANK_TRACK_WIDTH_M)

        # --- Pose estimator ---
        # DifferentialDrivePoseEstimator fuses wheel odometry with optional
        # vision measurements, giving the same interface as the swerve version.
        self._pose_estimator = DifferentialDrivePoseEstimator(
            self._kinematics,
            self._get_rotation2d(),
            0.0,
            0.0,
            Pose2d(),
        )

        # --- Field2d widget ---
        # Shows the robot's estimated pose on Elastic / Shuffleboard overlays
        # and populates the 3-D field view in AdvantageScope.
        self._field = wpilib.Field2d()
        wpilib.SmartDashboard.putData("Field", self._field)

        # --- AdvantageScope struct publishers ---
        # All topics live under /TankDrive/ so they don't collide with
        # /SwerveDrive/ if someone compares logs across robot types.
        #
        #   /TankDrive/Pose            → Pose2d struct (Odometry tab)
        #   /TankDrive/CommandedSpeeds → ChassisSpeeds struct (graph tab)
        #   /TankDrive/MeasuredSpeeds  → ChassisSpeeds back-computed from wheels
        #   /TankDrive/WheelSpeeds     → DifferentialDriveWheelSpeeds struct
        _nt = ntcore.NetworkTableInstance.getDefault()
        self._pose_pub = _nt.getStructTopic("/TankDrive/Pose", Pose2d).publish()
        self._commanded_speeds_pub = _nt.getStructTopic(
            "/TankDrive/CommandedSpeeds", ChassisSpeeds
        ).publish()
        self._measured_speeds_pub = _nt.getStructTopic(
            "/TankDrive/MeasuredSpeeds", ChassisSpeeds
        ).publish()
        self._wheel_speeds_pub = _nt.getStructTopic(
            "/TankDrive/WheelSpeeds", DifferentialDriveWheelSpeeds
        ).publish()

        # --- Internal state ---
        # Commanded chassis speeds for this loop — set by drive().
        self._chassis_speeds = ChassisSpeeds(0.0, 0.0, 0.0)
        # SysId override: when True, execute() skips normal closed-loop drive.
        self._sysid_active: bool = False
        # Wheel distances tracked for odometry (metres travelled since reset).
        self._left_dist_m: float = 0.0
        self._right_dist_m: float = 0.0

    # ------------------------------------------------------------------
    # Motor configuration (real robot only)
    # ------------------------------------------------------------------

    def _configure_motors(self) -> None:
        """Apply Phoenix 6 configuration to the four drive TalonFXs."""
        cfg = phoenix6.configs.TalonFXConfiguration()
        cfg.current_limits.stator_current_limit = _TANK_STATOR_LIMIT_A
        cfg.current_limits.stator_current_limit_enable = True

        # Right side is mechanically inverted — both sides drive forward
        # with positive duty cycle after applying the correct inversion.
        sides = [
            (
                (self._left_leader, self._left_follower),
                phoenix6.configs.config_groups.InvertedValue.COUNTER_CLOCKWISE_POSITIVE,
                _TANK_LEFT_IDS[0],
            ),
            (
                (self._right_leader, self._right_follower),
                phoenix6.configs.config_groups.InvertedValue.CLOCKWISE_POSITIVE,
                _TANK_RIGHT_IDS[0],
            ),
        ]
        for (leader, follower), inversion, leader_id in sides:
            cfg.motor_output.inverted = inversion
            leader.configurator.apply(cfg)
            follower.configurator.apply(cfg)
            follower.set_control(phoenix6.controls.Follower(leader_id, MotorAlignmentValue.ALIGNED))

    # ------------------------------------------------------------------
    # Public API — driving
    # ------------------------------------------------------------------

    def drive(self, vx: float, omega: float) -> None:
        """Command the drivetrain with a robot-relative chassis velocity.

        This is the primary drive method.  Call it from ``robot.py``'s
        ``teleopPeriodic()`` every loop.

        Parameters
        ----------
        vx:
            Forward velocity in metres/second (+x = forward).
        omega:
            Rotation rate in radians/second (CCW positive).
        """
        self._chassis_speeds = ChassisSpeeds(vx, 0.0, omega)

    def drive_arcade(self, throttle: float, turn: float) -> None:
        """Arcade-style drive — convenience wrapper around ``drive()``.

        Parameters
        ----------
        throttle:
            Forward demand, normalised [-1, 1].  Multiplied by
            ``_TANK_MAX_SPEED_MPS`` before being passed to ``drive()``.
        turn:
            Turn demand, normalised [-1, 1].  Multiplied by the maximum
            angular speed (2 × vMax / trackWidth) before being passed to
            ``drive()``.
        """
        vx = throttle * _TANK_MAX_SPEED_MPS
        max_omega = 2.0 * _TANK_MAX_SPEED_MPS / _TANK_TRACK_WIDTH_M
        self.drive(vx, turn * max_omega)

    def drive_tank(self, left: float, right: float) -> None:
        """Tank-style drive — convert left/right demands to ChassisSpeeds.

        Parameters
        ----------
        left:
            Left side demand, normalised [-1, 1].
        right:
            Right side demand, normalised [-1, 1].
        """
        left_mps = left * _TANK_MAX_SPEED_MPS
        right_mps = right * _TANK_MAX_SPEED_MPS
        wheel_speeds = DifferentialDriveWheelSpeeds(left_mps, right_mps)
        self._chassis_speeds = self._kinematics.toChassisSpeeds(wheel_speeds)

    def drive_chassis_speeds(self, speeds: ChassisSpeeds) -> None:
        """Accept a ``ChassisSpeeds`` command (e.g. from PathPlanner).

        Parameters
        ----------
        speeds:
            Robot-relative target chassis speeds.  Units: m/s (vx), rad/s (omega).
            The ``vy`` component is ignored for a differential drive.
        """
        self._chassis_speeds = speeds

    def stop(self) -> None:
        """Immediately command zero velocity."""
        self._chassis_speeds = ChassisSpeeds(0.0, 0.0, 0.0)

    # ------------------------------------------------------------------
    # Public API — odometry / pose
    # ------------------------------------------------------------------

    def get_pose(self) -> Pose2d:
        """Return the current estimated field pose from the pose estimator."""
        return self._pose_estimator.getEstimatedPosition()

    def reset_pose(self, pose: Pose2d) -> None:
        """Reset the pose estimator to a known pose (e.g. at match start)."""
        self._left_dist_m = 0.0
        self._right_dist_m = 0.0
        self._pose_estimator.resetPosition(
            self._get_rotation2d(),
            leftDistance=0.0,
            rightDistance=0.0,
            pose=pose,
        )

    def get_chassis_speeds(self) -> ChassisSpeeds:
        """Return the measured robot-relative chassis speeds from wheel encoders."""
        return self._kinematics.toChassisSpeeds(self._get_wheel_speeds())

    def add_vision_measurement(
        self,
        vision_pose: Pose2d,
        timestamp_s: float,
        std_devs: tuple[float, float, float],
    ) -> None:
        """Fuse a vision-derived pose estimate into the pose estimator.

        Identical interface to ``SwerveDrive.add_vision_measurement()`` so
        the ``Limelight`` component works with both drivetrain types.

        Parameters
        ----------
        vision_pose:
            Robot pose estimated from AprilTag vision in field coordinates.
        timestamp_s:
            FPGA timestamp (seconds) when the camera frame was captured,
            compensated for pipeline and capture latency.
        std_devs:
            Measurement noise standard deviations ``(x_m, y_m, heading_rad)``.
        """
        self._pose_estimator.addVisionMeasurement(vision_pose, timestamp_s)

    def get_field(self) -> wpilib.Field2d:
        """Return the shared ``Field2d`` widget.

        Used by ``Limelight`` to paint per-camera ghost poses on the field
        overlay — identical interface to ``SwerveDrive.get_field()``.
        """
        return self._field

    def get_heading_rad(self) -> float:
        """Return the current robot heading in radians (CCW positive)."""
        return math.radians(self._gyro.get_yaw().value)

    def reset_gyro(self) -> None:
        """Zero the gyro, treating the current heading as 0°."""
        self._gyro.set_yaw(0.0)

    # ------------------------------------------------------------------
    # Public API — SysId helpers
    # ------------------------------------------------------------------

    def set_open_loop_voltage(self, left_volts: float, right_volts: float) -> None:
        """Apply open-loop voltages to the left and right sides independently.

        Called by a SysId characterisation routine.  Sets ``_sysid_active``
        so ``execute()`` skips normal closed-loop drive.

        Parameters
        ----------
        left_volts:
            Voltage applied to the left side (+ve = forward).
        right_volts:
            Voltage applied to the right side (+ve = forward).
        """
        self._sysid_active = True
        if not wpilib.RobotBase.isSimulation():
            self._left_leader.set_control(phoenix6.controls.VoltageOut(left_volts))
            self._right_leader.set_control(phoenix6.controls.VoltageOut(right_volts))

    def stop_sysid(self) -> None:
        """End an open-loop SysId run and return to closed-loop drive."""
        self._sysid_active = False
        self.stop()

    def get_avg_drive_position_m(self) -> float:
        """Average driven distance across both sides (metres)."""
        return (self._left_dist_m + self._right_dist_m) / 2.0

    def get_avg_drive_velocity_mps(self) -> float:
        """Average wheel speed across both sides (m/s)."""
        ws = self._get_wheel_speeds()
        return (ws.left + ws.right) / 2.0

    # ------------------------------------------------------------------
    # MagicBot @feedback telemetry
    # Each method is called every loop and published to SmartDashboard
    # under "components/tank_drive/<method_name>".
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
    def left_velocity_mps(self) -> float:
        """Measured left-side wheel speed in metres/second."""
        return self._get_wheel_speeds().left

    @feedback
    def right_velocity_mps(self) -> float:
        """Measured right-side wheel speed in metres/second."""
        return self._get_wheel_speeds().right

    @feedback
    def velocity_x_mps(self) -> float:
        """Commanded forward velocity in metres/second."""
        return self._chassis_speeds.vx

    @feedback
    def angular_velocity_dps(self) -> float:
        """Commanded rotation rate in degrees/second."""
        return math.degrees(self._chassis_speeds.omega)

    # ------------------------------------------------------------------
    # MagicBot execute()
    # ------------------------------------------------------------------

    def execute(self) -> None:
        """Apply the commanded chassis speeds to the drive motors.

        Called automatically by MagicBot every robot loop (~20 ms).
        When ``_sysid_active`` is ``True``, skips normal closed-loop drive
        so that an external SysId routine can hold open-loop voltage control.
        """
        wheel_speeds = self._kinematics.toWheelSpeeds(self._chassis_speeds)
        # Desaturate so neither side exceeds the physical speed limit.
        wheel_speeds.desaturate(_TANK_MAX_SPEED_MPS)

        if not self._sysid_active and not wpilib.RobotBase.isSimulation():
            # Convert wheel speed (m/s) → motor velocity (rot/s).
            left_rps = wheel_speeds.left / _TANK_WHEEL_CIRCUMFERENCE_M * _TANK_GEAR_RATIO
            right_rps = wheel_speeds.right / _TANK_WHEEL_CIRCUMFERENCE_M * _TANK_GEAR_RATIO
            self._left_leader.set_control(phoenix6.controls.VelocityVoltage(left_rps))
            self._right_leader.set_control(phoenix6.controls.VelocityVoltage(right_rps))

        # --- Odometry update ---
        # Integrate wheel velocities to get distance travelled each loop.
        dt = constants.ROBOT_LOOP_PERIOD_S
        self._left_dist_m += wheel_speeds.left * dt
        self._right_dist_m += wheel_speeds.right * dt

        self._pose_estimator.update(
            self._get_rotation2d(),
            leftDistance=self._left_dist_m,
            rightDistance=self._right_dist_m,
        )

        # --- Field2d + AdvantageScope struct telemetry ---
        pose = self.get_pose()
        measured_speeds = self._get_wheel_speeds()
        self._field.setRobotPose(pose)

        # Pose2d struct → AdvantageScope Odometry tab / 3-D field view.
        self._pose_pub.set(pose)
        # ChassisSpeeds structs → graph tab (commanded vs. measured).
        self._commanded_speeds_pub.set(self._chassis_speeds)
        self._measured_speeds_pub.set(self._kinematics.toChassisSpeeds(measured_speeds))
        # DifferentialDriveWheelSpeeds struct → per-side velocity graph.
        self._wheel_speeds_pub.set(measured_speeds)

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _get_rotation2d(self) -> Rotation2d:
        """Read the gyro yaw and wrap it into a ``Rotation2d``."""
        return Rotation2d(self.get_heading_rad())

    def _get_wheel_speeds(self) -> DifferentialDriveWheelSpeeds:
        """Read the left/right wheel speeds from motor encoders (m/s).

        In simulation the motors are stubs that report zero velocity —
        the speeds returned here reflect whatever the physics simulation
        has written back into the stub's ``_velocity_rps`` field.
        """
        # Motor velocity is in rotations/second (Phoenix 6 native unit).
        # Convert: m/s = (rot/s) / gear_ratio × wheel_circumference
        ratio = _TANK_GEAR_RATIO * _TANK_WHEEL_CIRCUMFERENCE_M
        left_mps = self._left_leader.get_velocity().value / ratio
        right_mps = self._right_leader.get_velocity().value / ratio
        return DifferentialDriveWheelSpeeds(left_mps, right_mps)
