"""Intake component.

Controls the intake mechanism: one Falcon 500 (TalonFX) through a 5:1
planetary gearbox.  The gear ratio is defined in ``constants.INTAKE_GEAR_RATIO``
so it can be tuned without touching this file.

The component uses a simple state-machine pattern:
  - Driver code calls ``intake()``, ``eject()``, or ``stop()`` to set intent.
  - ``execute()`` applies that intent to the motor once per loop.
"""

from __future__ import annotations

import enum

import wpilib

if wpilib.RobotBase.isSimulation():
    from components.sim_devices import DutyCycleOut, NeutralOut, SimDigitalInput, SimTalonFX
else:
    import phoenix6.configs
    import phoenix6.hardware
    import phoenix6.signals
    from phoenix6.controls import DutyCycleOut, NeutralOut
from magicbot import feedback

import constants

# Imported at the bottom of the module (after Shooter is defined) to avoid
# the circular import: intake → shooter → (nothing), but shooter imports
# nothing from intake, so the real fix is a deferred module-level import.
# We use a local import inside the annotation so Python can resolve it at
# class-creation time when typing.get_type_hints() is called by MagicBot.
from components.shooter import Shooter, ShooterState  # noqa: E402


class IntakeState(enum.Enum):
    """Discrete operating states for the intake."""

    STOPPED = enum.auto()
    INTAKING = enum.auto()
    EJECTING = enum.auto()
    # Game piece detected — hold position, don't keep spinning.
    HOLDING = enum.auto()


class Intake:
    """MagicBot component — note intake with a single Falcon 500.

    Usage (from ``robot.py`` or a StateMachine)::

        self.intake.intake()   # start pulling game pieces in
        self.intake.eject()    # push game pieces out
        self.intake.stop()     # coast to stop

    Shooter interlock
    -----------------
    If the shooter is spinning but has NOT reached target speed, calling
    ``intake()`` will be silently blocked — the motor will not run.  This
    prevents jamming a game piece into a slow flywheel.  The interlock is
    bypassed during ejection (eject always works) and when the shooter is
    completely stopped (intaking without shooting is fine).

    MagicBot injects the shared ``Shooter`` instance automatically because
    both components are declared in ``robot.py``.
    """

    # MagicBot injects this from the shared Shooter component.
    shooter: Shooter

    def __init__(self) -> None:
        if wpilib.RobotBase.isSimulation():
            self._motor = SimTalonFX()
            # In sim, use a stub digital input so physics.py can trigger the
            # beam break by setting _beam_break._value = False.
            self._beam_break = SimDigitalInput(constants.INTAKE_BEAM_BREAK_PORT)
        else:
            self._motor = phoenix6.hardware.TalonFX(
                constants.INTAKE_MOTOR_ID, constants.CANIVORE_BUS
            )
            self._configure_motor()
            # Real robot: hardware beam break sensor on a DIO port.
            self._beam_break = wpilib.DigitalInput(constants.INTAKE_BEAM_BREAK_PORT)

        # Default state is stopped.
        self._state: IntakeState = IntakeState.STOPPED

    # ------------------------------------------------------------------
    # Motor configuration
    # ------------------------------------------------------------------

    def _configure_motor(self) -> None:
        """Apply current limits and neutral mode to the intake Falcon."""
        cfg = phoenix6.configs.TalonFXConfiguration()

        cfg.current_limits.stator_current_limit = constants.INTAKE_STATOR_CURRENT_LIMIT_A
        cfg.current_limits.stator_current_limit_enable = True
        cfg.current_limits.supply_current_limit = constants.INTAKE_SUPPLY_CURRENT_LIMIT_A
        cfg.current_limits.supply_current_limit_enable = True

        # Coast on neutral so the intake doesn't fight the game piece.
        cfg.motor_output.neutral_mode = phoenix6.signals.NeutralModeValue.COAST

        self._motor.configurator.apply(cfg)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def intake(self) -> None:
        """Set intent to pull game pieces into the robot."""
        self._state = IntakeState.INTAKING

    def eject(self) -> None:
        """Set intent to push game pieces out of the robot."""
        self._state = IntakeState.EJECTING

    def stop(self) -> None:
        """Set intent to stop the intake motor."""
        self._state = IntakeState.STOPPED

    @property
    def is_running(self) -> bool:
        """``True`` when the intake is actively intaking or ejecting."""
        return self._state not in (IntakeState.STOPPED, IntakeState.HOLDING)

    def has_game_piece(self) -> bool:
        """Return ``True`` when the beam break sensor detects a game piece.

        The beam break sensor reads ``False`` when the IR beam is interrupted
        (i.e. something is physically blocking it inside the intake tunnel).
        We invert the reading so that ``True`` means "game piece present",
        which is more intuitive for callers.

        In simulation the sensor always reads ``True`` (beam clear), so
        ``has_game_piece()`` always returns ``False`` in sim.
        """
        return not self._beam_break.get()

    # ------------------------------------------------------------------
    # Telemetry — published to NetworkTables via MagicBot @feedback
    # ------------------------------------------------------------------

    @feedback
    def state_name(self) -> str:
        """Current intake state as a human-readable string."""
        return self._state.name

    @feedback
    def game_piece_detected(self) -> bool:
        """``True`` when the beam break sensor detects a game piece."""
        return self.has_game_piece()

    @feedback
    def motor_duty_cycle(self) -> float:
        """Applied motor duty cycle (-1.0 to 1.0). Positive = intaking."""
        return self._motor.get_duty_cycle().value

    @feedback
    def supply_current_amps(self) -> float:
        """Intake motor supply current draw in amps."""
        return self._motor.get_supply_current().value

    # ------------------------------------------------------------------
    # MagicBot interface
    # ------------------------------------------------------------------

    def execute(self) -> None:
        """Apply the current intake state to the motor.

        Called automatically by MagicBot every robot loop.

        Auto-stop logic: if we are intaking and the beam break detects a
        game piece, transition to HOLDING immediately.  This prevents the
        motor from fighting against a captured game piece and gives the
        driver a clear "I have a piece" signal via telemetry.

        The HOLDING state keeps the motor stopped until the driver explicitly
        calls ``eject()`` or the game piece is no longer detected.
        """
        # Auto-transition: beam break tripped while intaking → hold.
        if self._state is IntakeState.INTAKING and self.has_game_piece():
            self._state = IntakeState.HOLDING

        # If we had a game piece but it's gone (e.g. was shot), leave HOLDING.
        if self._state is IntakeState.HOLDING and not self.has_game_piece():
            self._state = IntakeState.STOPPED

        # Shooter interlock: if the shooter is spinning but not yet at speed,
        # block intake to prevent jamming a game piece into a slow flywheel.
        # Ejection always works.  Intaking with the shooter stopped is fine.
        shooter_spinning_not_ready = (
            self.shooter._state is not ShooterState.STOPPED and not self.shooter.is_at_speed()
        )
        if self._state is IntakeState.INTAKING and shooter_spinning_not_ready:
            self._state = IntakeState.STOPPED

        match self._state:
            case IntakeState.INTAKING:
                self._motor.set_control(DutyCycleOut(constants.INTAKE_SPEED_PERCENT))
            case IntakeState.EJECTING:
                self._motor.set_control(DutyCycleOut(constants.INTAKE_EJECT_SPEED_PERCENT))
            case IntakeState.STOPPED | IntakeState.HOLDING:
                self._motor.set_control(NeutralOut())

        # Reset to STOPPED each loop so the button must be held continuously.
        # HOLDING persists until explicitly cleared (see auto-transition above).
        if self._state is not IntakeState.HOLDING:
            self._state = IntakeState.STOPPED
