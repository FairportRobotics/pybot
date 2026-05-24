"""LED subsystem component — REV Blinkin LED driver.

Reads the ``ScoringStateMachine`` and ``Shooter`` state each loop and
commands a REV Blinkin LED driver (connected to a PWM servo port) to show
the appropriate colour/pattern to the drivers.

Pattern selection priority (highest first)
------------------------------------------
1. **FIRING**      — strobe red       (active shot in progress)
2. **READY**       — solid orange     (piece held AND flywheel at speed)
3. **HOLDING**     — solid green      (piece held, flywheel spinning up)
4. **SPINNING UP** — fast strobe yellow (flywheel spinning, no piece yet)
5. **INTAKING**    — strobe white     (intake running, waiting for piece)
6. **IDLE**        — solid white      (ready, nothing happening)

Blinkin wiring
--------------
* Wire the Blinkin signal wire to a roboRIO PWM port.
* Set ``constants.BLINKIN_PWM_PORT`` to match the port number.
* The Blinkin is powered from the robot power distribution (12 V).

Pattern values
--------------
All pattern values are in ``constants.py`` (``BLINKIN_*``).  Consult the
REV Blinkin pattern table PDF to select the patterns you want.

Simulation
----------
A ``SimServo`` stub is used in simulation so the component runs without
error.  The commanded value is visible in NetworkTables via ``@feedback``.
"""

from __future__ import annotations

import wpilib

if wpilib.RobotBase.isSimulation():
    from components.sim_devices import SimServo

# Forward-declare ScoringStateMachine to avoid a circular import at module
# load time.  MagicBot resolves the type annotation at runtime.
from magicbot import feedback

import constants
from components.shooter import Shooter
from state_machine.scoring import ScoringStateMachine


class LEDSubsystem:
    """MagicBot component — drives a REV Blinkin LED strip controller.

    MagicBot injects ``shooter`` and ``scoring`` automatically because their
    names match the robot-level component declarations.
    """

    # Injected by MagicBot.
    shooter: Shooter
    scoring: ScoringStateMachine

    def setup(self) -> None:
        """Create the Blinkin servo output after injection."""
        if wpilib.RobotBase.isSimulation():
            self._blinkin = SimServo(constants.BLINKIN_PWM_PORT)
        else:
            self._blinkin = wpilib.Servo(constants.BLINKIN_PWM_PORT)

        self._pattern: float = constants.BLINKIN_IDLE

    # ------------------------------------------------------------------
    # Telemetry
    # ------------------------------------------------------------------

    @feedback
    def pattern_value(self) -> float:
        """Current Blinkin PWM pattern value (0.0–1.0)."""
        return self._pattern

    @feedback
    def pattern_name(self) -> str:
        """Human-readable name of the active LED pattern."""
        mapping = {
            constants.BLINKIN_IDLE: "Idle (white)",
            constants.BLINKIN_INTAKING: "Intaking (strobe white)",
            constants.BLINKIN_HOLDING: "Holding (green)",
            constants.BLINKIN_SPINNING_UP: "Spinning Up (yellow)",
            constants.BLINKIN_READY_TO_FIRE: "Ready to Fire (orange)",
            constants.BLINKIN_FIRING: "Firing (strobe red)",
        }
        return mapping.get(self._pattern, f"Unknown ({self._pattern:.3f})")

    # ------------------------------------------------------------------
    # MagicBot execute()
    # ------------------------------------------------------------------

    def execute(self) -> None:
        """Select and apply the highest-priority LED pattern each loop."""
        state = self.scoring.current_state

        if state == "firing":
            self._pattern = constants.BLINKIN_FIRING
        elif state == "holding" and self.shooter.is_at_speed():
            self._pattern = constants.BLINKIN_READY_TO_FIRE
        elif state == "holding":
            self._pattern = constants.BLINKIN_HOLDING
        elif state == "intaking" and self.shooter.state_name != "STOPPED":
            self._pattern = constants.BLINKIN_SPINNING_UP
        elif state == "intaking":
            self._pattern = constants.BLINKIN_INTAKING
        else:
            self._pattern = constants.BLINKIN_IDLE

        self._blinkin.set(self._pattern)
