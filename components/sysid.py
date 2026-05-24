"""Drivetrain System Identification (SysId) component.

Implements the four standard WPILib SysId routines for a swerve drivetrain:

  1. **Quasistatic Forward** — slowly ramp voltage from 0 towards the maximum
     while driving forward; captures steady-state velocity vs voltage data.
  2. **Quasistatic Reverse** — same, but in reverse.
  3. **Dynamic Forward** — apply a fixed step voltage and record the dynamic
     acceleration response (forward).
  4. **Dynamic Reverse** — same, but in reverse.

Usage
-----
Enable **Test mode** on the Driver Station, then hold the corresponding
Xbox controller button for the desired routine:

  * **A** — Quasistatic Forward
  * **B** — Quasistatic Reverse
  * **X** — Dynamic Forward
  * **Y** — Dynamic Reverse

Release the button to stop the routine and return to normal drive.

Data Logging
------------
All data is written to a WPILib ``DataLog`` (`.wpilog`) file via
``wpilib.DataLogManager``.  Start the Data Log Manager in ``testInit()``
in ``robot.py`` (already done).

After running the routines, copy the ``.wpilog`` file from the robot (or
from the simulation working directory in sim) and open it in the
**WPILib SysId Analysis Tool** (part of the WPILib suite):

  ``Tools → SysId``

Log entry names follow the format expected by the SysId tool:

  * ``sysid-test-state-{mechanism}`` — active routine name (string)
  * ``sysid-{mechanism}-volts``       — applied voltage (double, V)
  * ``sysid-{mechanism}-position``    — average drive position (double, m)
  * ``sysid-{mechanism}-velocity``    — average drive velocity (double, m/s)

Safety
------
* Only runs in DS Test mode — MagicBot routes to ``testPeriodic()``.
* Motor voltage is capped at ``constants.SYSID_MAX_VOLTAGE_V``.
* Releasing the button immediately stops the routine and idles the drive.
* ``SwerveDrive._sysid_active`` prevents the normal drive loop from
  overriding open-loop voltage during an active routine.
"""

from __future__ import annotations

import enum

import wpilib
import wpiutil.log
from magicbot import feedback

import constants
from components.swerve_drive import SwerveDrive


class SysIdState(enum.Enum):
    """Active SysId routine."""

    IDLE = enum.auto()
    QUASISTATIC_FORWARD = enum.auto()
    QUASISTATIC_REVERSE = enum.auto()
    DYNAMIC_FORWARD = enum.auto()
    DYNAMIC_REVERSE = enum.auto()


# Human-readable state names written to the log (must match SysId tool format).
_STATE_NAMES: dict[SysIdState, str] = {
    SysIdState.IDLE: "none",
    SysIdState.QUASISTATIC_FORWARD: "quasistatic-forward",
    SysIdState.QUASISTATIC_REVERSE: "quasistatic-reverse",
    SysIdState.DYNAMIC_FORWARD: "dynamic-forward",
    SysIdState.DYNAMIC_REVERSE: "dynamic-reverse",
}


class DrivetrainSysId:
    """MagicBot component — drivetrain system identification.

    MagicBot injects the ``swerve_drive`` component automatically.
    This component is a no-op when ``_state == IDLE`` so it does not
    interfere with normal teleop or autonomous operation.
    """

    # Injected by MagicBot from the shared swerve_drive component.
    swerve_drive: SwerveDrive

    def setup(self) -> None:
        """Initialise DataLog entries after MagicBot injection is complete."""
        log = wpilib.DataLogManager.getLog()
        mech = constants.SYSID_MECHANISM_NAME

        self._log_state = wpiutil.log.StringLogEntry(log, f"sysid-test-state-{mech}")
        self._log_volts = wpiutil.log.DoubleLogEntry(log, f"sysid-{mech}-volts")
        self._log_pos = wpiutil.log.DoubleLogEntry(log, f"sysid-{mech}-position")
        self._log_vel = wpiutil.log.DoubleLogEntry(log, f"sysid-{mech}-velocity")

        self._state: SysIdState = SysIdState.IDLE
        self._elapsed_s: float = 0.0
        self._applied_volts: float = 0.0

    # ------------------------------------------------------------------
    # Public API — called from robot.py testPeriodic()
    # ------------------------------------------------------------------

    def quasistatic_forward(self) -> None:
        """Start or continue a quasistatic forward routine."""
        self._state = SysIdState.QUASISTATIC_FORWARD

    def quasistatic_reverse(self) -> None:
        """Start or continue a quasistatic reverse routine."""
        self._state = SysIdState.QUASISTATIC_REVERSE

    def dynamic_forward(self) -> None:
        """Start or continue a dynamic (step) forward routine."""
        self._state = SysIdState.DYNAMIC_FORWARD

    def dynamic_reverse(self) -> None:
        """Start or continue a dynamic (step) reverse routine."""
        self._state = SysIdState.DYNAMIC_REVERSE

    # ------------------------------------------------------------------
    # Telemetry
    # ------------------------------------------------------------------

    @feedback
    def sysid_state(self) -> str:
        """Name of the currently active SysId routine."""
        return _STATE_NAMES[self._state]

    @feedback
    def sysid_voltage(self) -> float:
        """Voltage being applied to the drive motors this loop (V)."""
        return self._applied_volts

    @feedback
    def sysid_position_m(self) -> float:
        """Average drive position across all modules (m)."""
        return self.swerve_drive.get_avg_drive_position_m()

    @feedback
    def sysid_velocity_mps(self) -> float:
        """Average drive velocity across all modules (m/s)."""
        return self.swerve_drive.get_avg_drive_velocity_mps()

    # ------------------------------------------------------------------
    # MagicBot interface
    # ------------------------------------------------------------------

    def execute(self) -> None:
        """Apply the active SysId routine for this loop.

        Called automatically by MagicBot every robot loop (all modes).
        In IDLE state this is a no-op.
        """
        if self._state is SysIdState.IDLE:
            # Ensure the drive reverts to normal control.
            self.swerve_drive.stop_sysid()
            self._elapsed_s = 0.0
            self._applied_volts = 0.0
            return

        dt = constants.ROBOT_LOOP_PERIOD_S  # 20 ms nominal loop period
        self._elapsed_s += dt

        # --- Compute voltage for this routine ---
        match self._state:
            case SysIdState.QUASISTATIC_FORWARD:
                volts = constants.SYSID_QUASISTATIC_RAMP_RATE_V_PER_S * self._elapsed_s
            case SysIdState.QUASISTATIC_REVERSE:
                volts = -(constants.SYSID_QUASISTATIC_RAMP_RATE_V_PER_S * self._elapsed_s)
            case SysIdState.DYNAMIC_FORWARD:
                volts = constants.SYSID_DYNAMIC_STEP_VOLTAGE_V
            case SysIdState.DYNAMIC_REVERSE:
                volts = -constants.SYSID_DYNAMIC_STEP_VOLTAGE_V
            case _:
                volts = 0.0

        # Clamp to safety limit.
        volts = max(-constants.SYSID_MAX_VOLTAGE_V, min(constants.SYSID_MAX_VOLTAGE_V, volts))
        self._applied_volts = volts

        # Apply voltage via SwerveDrive open-loop override.
        self.swerve_drive.set_open_loop_voltage(volts)

        # --- Log to DataLog ---
        now_us = int(wpilib.Timer.getFPGATimestamp() * 1e6)
        self._log_state.append(_STATE_NAMES[self._state], now_us)
        self._log_volts.append(volts, now_us)
        self._log_pos.append(self.swerve_drive.get_avg_drive_position_m(), now_us)
        self._log_vel.append(self.swerve_drive.get_avg_drive_velocity_mps(), now_us)

        # Reset to IDLE so the button must be held continuously.
        self._state = SysIdState.IDLE
