"""Shooter component.

Controls the single-wheel flywheel shooter powered by one Falcon 500 (TalonFX).
Velocity closed-loop control (Phoenix 6 slot 0) is used to hold a target
rotations-per-second set point.

The target RPM and PID gains live in ``constants.py`` so they can be tuned
without touching this file.

State-machine pattern
---------------------
- ``spin_up()``  — ramp the flywheel to the target speed.
- ``stop()``     — coast the flywheel to rest.
- ``is_at_speed()`` — query whether the flywheel is within tolerance.
- ``execute()``  — apply the commanded state to the motor each loop.
"""

from __future__ import annotations

import enum

import wpilib

if wpilib.RobotBase.isSimulation():
    from components.sim_devices import NeutralOut, SimTalonFX, VelocityVoltage
else:
    import phoenix6.configs
    import phoenix6.hardware
    import phoenix6.signals
    from phoenix6.controls import NeutralOut, VelocityVoltage
from magicbot import feedback

import constants


class ShooterState(enum.Enum):
    """Discrete operating states for the shooter flywheel."""

    STOPPED = enum.auto()
    SPINNING_UP = enum.auto()


class Shooter:
    """MagicBot component — single-wheel flywheel shooter.

    Usage (from ``robot.py`` or a StateMachine)::

        self.shooter.spin_up()      # start accelerating flywheel
        self.shooter.set_speed(60)  # override target speed (rot/s)
        self.shooter.stop()         # let flywheel coast down

        if self.shooter.is_at_speed():
            # safe to feed game piece
    """

    def __init__(self) -> None:
        if wpilib.RobotBase.isSimulation():
            self._motor = SimTalonFX()
        else:
            self._motor = phoenix6.hardware.TalonFX(
                constants.SHOOTER_MOTOR_ID, constants.CANIVORE_BUS
            )
            self._configure_motor()

        self._state: ShooterState = ShooterState.STOPPED
        self._target_rps: float = constants.SHOOTER_TARGET_RPS

    # ------------------------------------------------------------------
    # Motor configuration
    # ------------------------------------------------------------------

    def _configure_motor(self) -> None:
        """Apply velocity PID gains, current limits, and neutral mode."""
        cfg = phoenix6.configs.TalonFXConfiguration()

        cfg.current_limits.stator_current_limit = constants.SHOOTER_STATOR_CURRENT_LIMIT_A
        cfg.current_limits.stator_current_limit_enable = True
        cfg.current_limits.supply_current_limit = constants.SHOOTER_SUPPLY_CURRENT_LIMIT_A
        cfg.current_limits.supply_current_limit_enable = True

        # Coast neutral so the flywheel spins down naturally when stopped.
        cfg.motor_output.neutral_mode = phoenix6.signals.NeutralModeValue.COAST

        # Velocity PID — slot 0
        cfg.slot0.k_p = constants.SHOOTER_KP
        cfg.slot0.k_i = constants.SHOOTER_KI
        cfg.slot0.k_d = constants.SHOOTER_KD
        cfg.slot0.k_s = constants.SHOOTER_KS
        cfg.slot0.k_v = constants.SHOOTER_KV

        self._motor.configurator.apply(cfg)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def spin_up(self) -> None:
        """Command the flywheel to accelerate to the target speed."""
        self._state = ShooterState.SPINNING_UP

    def stop(self) -> None:
        """Let the flywheel coast to rest."""
        self._state = ShooterState.STOPPED

    def set_speed(self, target_rps: float) -> None:
        """Override the flywheel target speed for this loop.

        Parameters
        ----------
        target_rps:
            Desired flywheel velocity in rotations per second.
        """
        self._target_rps = target_rps
        self._state = ShooterState.SPINNING_UP

    def reset_speed(self) -> None:
        """Restore the flywheel target speed to the default from constants."""
        self._target_rps = constants.SHOOTER_TARGET_RPS

    def is_at_speed(self) -> bool:
        """Return ``True`` when the flywheel is within tolerance of target.

        Use this as a gate before feeding the game piece into the shooter.
        """
        if self._state is ShooterState.STOPPED:
            return False
        error = abs(self._motor.get_velocity().value - self._target_rps)
        return error <= constants.SHOOTER_AT_SPEED_TOLERANCE_RPS

    @property
    def current_speed_rps(self) -> float:
        """Current measured flywheel velocity in rotations/second."""
        return self._motor.get_velocity().value

    # ------------------------------------------------------------------
    # Telemetry — published to NetworkTables via MagicBot @feedback
    # ------------------------------------------------------------------

    @feedback
    def flywheel_speed_rpm(self) -> float:
        """Current flywheel speed in RPM (rotations per minute)."""
        return self._motor.get_velocity().value * 60.0

    @feedback
    def target_speed_rpm(self) -> float:
        """Target flywheel speed in RPM."""
        return self._target_rps * 60.0

    @feedback
    def speed_error_rpm(self) -> float:
        """Difference between target and actual speed in RPM (0 = on target)."""
        return (self._target_rps - self._motor.get_velocity().value) * 60.0

    @feedback
    def at_speed(self) -> bool:
        """``True`` when the flywheel is within tolerance of the target speed."""
        return self.is_at_speed()

    @feedback
    def state_name(self) -> str:
        """Current shooter state as a human-readable string."""
        return self._state.name

    @feedback
    def supply_current_amps(self) -> float:
        """Shooter motor supply current draw in amps."""
        return self._motor.get_supply_current().value

    # ------------------------------------------------------------------
    # MagicBot interface
    # ------------------------------------------------------------------

    def execute(self) -> None:
        """Apply the current shooter state to the motor.

        Called automatically by MagicBot every robot loop.
        """
        match self._state:
            case ShooterState.SPINNING_UP:
                self._motor.set_control(VelocityVoltage(self._target_rps, slot=0, enable_foc=False))
            case ShooterState.STOPPED:
                self._motor.set_control(NeutralOut())
