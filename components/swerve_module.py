"""SwerveModule component.

Encapsulates one SDS MK4i swerve module consisting of:
  - One Kraken X60 (TalonFX) for driving
  - One Kraken X60 (TalonFX) for steering (azimuth)
  - One CANcoder for absolute steering angle

All Phoenix 6 devices communicate over the CANivore CAN bus defined in
``constants.CANIVORE_BUS``.

The module uses:
  - Velocity closed-loop control (slot 0) for the drive motor.
  - Position closed-loop control (slot 0) for the steer motor, with the
    CANcoder fused as the remote sensor so the absolute position is always
    known without homing.
"""

from __future__ import annotations

import math

import wpilib

if wpilib.RobotBase.isSimulation():
    from components.sim_devices import (
        NeutralOut,
        PositionVoltage,
        SimCANcoder,
        SimTalonFX,
        VelocityVoltage,
        VoltageOut,
    )
else:
    import phoenix6
    import phoenix6.configs
    import phoenix6.hardware
    import phoenix6.signals
    from phoenix6.controls import (  # noqa: F401
        NeutralOut,
        PositionVoltage,
        VelocityVoltage,
        VoltageOut,
    )
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModulePosition, SwerveModuleState

import constants


class SwerveModule:
    """Represents one corner of the swerve drivetrain.

    MagicBot injects this component into ``SwerveDrive``; it is NOT itself a
    top-level component (no ``execute`` called by the framework directly).
    Instead, ``SwerveDrive.execute()`` calls ``apply_state()`` on each module.
    """

    def __init__(
        self,
        drive_id: int,
        steer_id: int,
        cancoder_id: int,
        cancoder_offset_rad: float,
        drive_inverted: bool = False,
    ) -> None:
        """Configure one swerve module.

        Parameters
        ----------
        drive_id:
            CAN ID of the drive TalonFX (Kraken X60).
        steer_id:
            CAN ID of the steer TalonFX (Kraken X60).
        cancoder_id:
            CAN ID of the CANcoder absolute encoder.
        cancoder_offset_rad:
            Absolute offset (radians) to zero the module when wheels face
            forward.  Measured empirically after installation.
        drive_inverted:
            Set ``True`` if the drive motor runs backwards for forward motion.
        """
        if wpilib.RobotBase.isSimulation():
            # Use lightweight stubs in simulation to avoid the standalone
            # phoenix6 package's C-level sim thread crashing under pyfrc.
            self._drive = SimTalonFX()
            self._steer = SimTalonFX()
            self._cancoder = SimCANcoder(cancoder_id)
        else:
            self._drive = phoenix6.hardware.TalonFX(drive_id, constants.CANIVORE_BUS)
            self._steer = phoenix6.hardware.TalonFX(steer_id, constants.CANIVORE_BUS)
            self._cancoder = phoenix6.hardware.CANcoder(cancoder_id, constants.CANIVORE_BUS)
            self._configure_cancoder(cancoder_offset_rad)
            self._configure_drive(drive_inverted)
            self._configure_steer()

        # Desired state set each loop; applied inside apply_state().
        self._desired_state: SwerveModuleState = SwerveModuleState(0.0, Rotation2d(0.0))

    # ------------------------------------------------------------------
    # Motor / sensor configuration helpers
    # ------------------------------------------------------------------

    def _configure_cancoder(self, offset_rad: float) -> None:
        """Apply factory defaults then set the magnet offset."""
        cfg = phoenix6.configs.CANcoderConfiguration()
        cfg.magnet_sensor.magnet_offset = offset_rad / (2 * math.pi)  # convert to rotations
        cfg.magnet_sensor.sensor_direction = (
            phoenix6.signals.SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE
        )
        cfg.magnet_sensor.absolute_sensor_discontinuity_point = (
            constants.CANCODER_DISCONTINUITY_POINT
        )
        self._cancoder.configurator.apply(cfg)

    def _configure_drive(self, inverted: bool) -> None:
        """Configure the drive TalonFX for velocity closed-loop control."""
        cfg = phoenix6.configs.TalonFXConfiguration()

        # Current limits
        cfg.current_limits.stator_current_limit = constants.DRIVE_STATOR_CURRENT_LIMIT_A
        cfg.current_limits.stator_current_limit_enable = True
        cfg.current_limits.supply_current_limit = constants.DRIVE_SUPPLY_CURRENT_LIMIT_A
        cfg.current_limits.supply_current_limit_enable = True

        # Motor output direction and neutral mode
        cfg.motor_output.inverted = (
            phoenix6.signals.InvertedValue.CLOCKWISE_POSITIVE
            if inverted
            else phoenix6.signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        )
        cfg.motor_output.neutral_mode = phoenix6.signals.NeutralModeValue.BRAKE

        # Velocity PID — slot 0
        cfg.slot0.k_p = constants.DRIVE_KP
        cfg.slot0.k_i = constants.DRIVE_KI
        cfg.slot0.k_d = constants.DRIVE_KD
        cfg.slot0.k_s = constants.DRIVE_KS
        cfg.slot0.k_v = constants.DRIVE_KV

        # Feedback: integrated sensor (no external encoder needed for drive)
        cfg.feedback.feedback_sensor_source = (
            phoenix6.signals.FeedbackSensorSourceValue.ROTOR_SENSOR
        )

        self._drive.configurator.apply(cfg)

    def _configure_steer(self) -> None:
        """Configure the steer TalonFX with CANcoder fused feedback."""
        cfg = phoenix6.configs.TalonFXConfiguration()

        # Current limits
        cfg.current_limits.stator_current_limit = constants.STEER_STATOR_CURRENT_LIMIT_A
        cfg.current_limits.stator_current_limit_enable = True
        cfg.current_limits.supply_current_limit = constants.STEER_SUPPLY_CURRENT_LIMIT_A
        cfg.current_limits.supply_current_limit_enable = True

        # Steer motors should not back-drive; keep them in brake mode.
        cfg.motor_output.neutral_mode = phoenix6.signals.NeutralModeValue.BRAKE
        cfg.motor_output.inverted = phoenix6.signals.InvertedValue.CLOCKWISE_POSITIVE

        # Position PID — slot 0
        cfg.slot0.k_p = constants.STEER_KP
        cfg.slot0.k_i = constants.STEER_KI
        cfg.slot0.k_d = constants.STEER_KD
        cfg.slot0.k_s = constants.STEER_KS
        cfg.slot0.k_v = constants.STEER_KV

        # Fuse CANcoder as the position feedback source.
        cfg.feedback.feedback_sensor_source = (
            phoenix6.signals.FeedbackSensorSourceValue.FUSED_CANCODER
        )
        cfg.feedback.feedback_remote_sensor_id = self._cancoder.device_id
        cfg.feedback.sensor_to_mechanism_ratio = constants.STEER_SENSOR_TO_MECHANISM_RATIO
        cfg.feedback.rotor_to_sensor_ratio = constants.STEER_GEAR_RATIO

        # Allow continuous wrap so the steer never takes the long way around.
        cfg.closed_loop_general.continuous_wrap = True

        self._steer.configurator.apply(cfg)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def set_desired_state(self, state: SwerveModuleState) -> None:
        """Queue a new desired ``SwerveModuleState`` for this module.

        The state is optimised (shortest-path steering) before being stored.
        ``apply_state()`` must be called each loop to push it to the motors.

        Parameters
        ----------
        state:
            Target drive speed (m/s) and steer angle (``Rotation2d``).
        """
        current_angle = Rotation2d(self._get_steer_angle_rad())
        # optimize() mutates `state` in-place in this version of wpimath and returns None.
        SwerveModuleState.optimize(state, current_angle)
        self._desired_state = state

    def apply_state(self) -> None:
        """Send the latest desired state to both motors.

        Call this once per robot loop from ``SwerveDrive.execute()``.
        """
        # --- Drive ---
        # Convert m/s → motor rotations/s via wheel circumference and gear ratio.
        drive_rps = (
            self._desired_state.speed / constants.WHEEL_CIRCUMFERENCE_M * constants.DRIVE_GEAR_RATIO
        )
        self._drive.set_control(
            VelocityVoltage(drive_rps, slot=constants.DRIVE_PID_SLOT, enable_foc=False)
        )

        # --- Steer ---
        # The fused CANcoder feedback is in rotations (one full rotation = 1.0).
        steer_rotations = self._desired_state.angle.radians() / (2 * math.pi)
        self._steer.set_control(
            PositionVoltage(steer_rotations, slot=constants.STEER_PID_SLOT, enable_foc=False)
        )

    def stop(self) -> None:
        """Command both motors to neutral output."""
        self._drive.set_control(NeutralOut())
        self._steer.set_control(NeutralOut())

    def set_drive_voltage(self, volts: float) -> None:
        """Apply a fixed open-loop voltage to the drive motor (SysId use only).

        All modules are assumed to be aligned forward before calling this.
        The steer motor holds its current position.

        Parameters
        ----------
        volts:
            Voltage to apply, clamped to ±12 V.  Positive = forward.
        """
        volts = max(-constants.BATTERY_VOLTAGE_V, min(constants.BATTERY_VOLTAGE_V, volts))
        self._drive.set_control(VoltageOut(volts))

    def get_drive_position_m(self) -> float:
        """Return integrated drive position in metres."""
        rotations = self._drive.get_position().value
        return rotations / constants.DRIVE_GEAR_RATIO * constants.WHEEL_CIRCUMFERENCE_M

    def get_drive_velocity_mps(self) -> float:
        """Return drive velocity in metres/second."""
        rps = self._drive.get_velocity().value
        return rps / constants.DRIVE_GEAR_RATIO * constants.WHEEL_CIRCUMFERENCE_M

    def get_state(self) -> SwerveModuleState:
        """Return the current measured ``SwerveModuleState``.

        Drive velocity is converted from motor rotations/s back to m/s.
        """
        drive_rps = self._drive.get_velocity().value
        speed_mps = drive_rps / constants.DRIVE_GEAR_RATIO * constants.WHEEL_CIRCUMFERENCE_M
        angle = Rotation2d(self._get_steer_angle_rad())
        return SwerveModuleState(speed_mps, angle)

    def get_position(self) -> SwerveModulePosition:
        """Return the current ``SwerveModulePosition`` for odometry updates.

        Drive distance is converted from motor rotations to metres.
        """
        drive_rotations = self._drive.get_position().value
        distance_m = drive_rotations / constants.DRIVE_GEAR_RATIO * constants.WHEEL_CIRCUMFERENCE_M
        angle = Rotation2d(self._get_steer_angle_rad())
        return SwerveModulePosition(distance_m, angle)

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _get_steer_angle_rad(self) -> float:
        """Read the current absolute steer angle in radians from the CANcoder."""
        rotations = self._cancoder.get_absolute_position().value
        return rotations * 2 * math.pi
