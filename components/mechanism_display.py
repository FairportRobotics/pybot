"""Mechanism2d display component.

Publishes a WPILib ``Mechanism2d`` widget to SmartDashboard so the Elastic
dashboard (or Shuffleboard / Glass) can show an animated side-view of the
robot's intake and shooter state in real time — both in simulation and on
the real robot.

Layout (viewed from the right side of the robot)
-------------------------------------------------
::

    ┌─────────────────────────────────┐  ← canvas (3 m wide, 3 m tall)
    │                                 │
    │          [SHOOTER ARM]          │  ← ligament rotates to show speed
    │                ╲               │
    │                 ╲              │
    │    [ROOT]────────●──────────── │  ← fixed base
    │                 ╱              │
    │                ╱               │
    │          [INTAKE ARM]           │  ← ligament rotates to show state
    │                                 │
    └─────────────────────────────────┘

The ``intake_arm`` ligament colour changes with state:
  - Green  = intaking / holding game piece
  - Orange = ejecting
  - Grey   = stopped

The ``shooter_arm`` ligament colour changes with shooter state:
  - Red    = spinning up (below target speed)
  - Green  = at target speed
  - Grey   = stopped

Both ligaments extend/retract to give a visual length cue for speed/state.
"""

from __future__ import annotations

import wpilib
from wpilib import Mechanism2d, MechanismLigament2d, MechanismRoot2d

import constants
from components.intake import Intake, IntakeState
from components.shooter import Shooter, ShooterState


class MechanismDisplay:
    """MagicBot component — Mechanism2d visualisation for intake and shooter.

    MagicBot injects ``intake`` and ``shooter`` automatically.
    """

    # Injected by MagicBot.
    intake: Intake
    shooter: Shooter

    def setup(self) -> None:
        """Build the Mechanism2d structure after injection."""
        # Canvas: 3 m wide × 3 m tall — large enough to see the arms.
        self._mechanism = Mechanism2d(3.0, 3.0)

        # Root node at the centre of the canvas.
        root: MechanismRoot2d = self._mechanism.getRoot("Robot", 1.5, 1.5)

        # Intake arm — points downward initially (270°).
        self._intake_arm: MechanismLigament2d = root.appendLigament(
            "IntakeArm",
            length=0.4,
            angle=270.0,
            lineWidth=6,
            color=wpilib.Color8Bit(128, 128, 128),  # grey = stopped
        )

        # Shooter arm — points upward initially (90°).
        self._shooter_arm: MechanismLigament2d = root.appendLigament(
            "ShooterArm",
            length=0.4,
            angle=90.0,
            lineWidth=6,
            color=wpilib.Color8Bit(128, 128, 128),  # grey = stopped
        )

        wpilib.SmartDashboard.putData("Mechanism", self._mechanism)

    # ------------------------------------------------------------------
    # MagicBot execute()
    # ------------------------------------------------------------------

    def execute(self) -> None:
        """Update arm colours and lengths each loop."""
        self._update_intake_arm()
        self._update_shooter_arm()

    def _update_intake_arm(self) -> None:
        """Colour and length of the intake arm reflect IntakeState."""
        state = self.intake._state
        match state:
            case IntakeState.INTAKING:
                color = wpilib.Color8Bit(0, 200, 0)  # green
                length = 0.55
            case IntakeState.HOLDING:
                color = wpilib.Color8Bit(0, 255, 150)  # bright green
                length = 0.5
            case IntakeState.EJECTING:
                color = wpilib.Color8Bit(255, 140, 0)  # orange
                length = 0.45
            case _:  # STOPPED
                color = wpilib.Color8Bit(128, 128, 128)  # grey
                length = 0.4

        self._intake_arm.setLength(length)
        self._intake_arm.setColor(color)

    def _update_shooter_arm(self) -> None:
        """Colour and length of the shooter arm reflect ShooterState + speed."""
        state = self.shooter._state
        if state is ShooterState.STOPPED:
            color = wpilib.Color8Bit(128, 128, 128)  # grey
            length = 0.4
        elif self.shooter.is_at_speed():
            color = wpilib.Color8Bit(0, 200, 0)  # green = ready
            length = 0.65
        else:
            # Spinning up — lerp red→yellow based on speed fraction.
            fraction = min(
                1.0,
                self.shooter.current_speed_rps / max(constants.SHOOTER_TARGET_RPS, 1e-6),
            )
            red = 255
            green = int(200 * fraction)
            color = wpilib.Color8Bit(red, green, 0)
            length = 0.4 + 0.25 * fraction

        self._shooter_arm.setLength(length)
        self._shooter_arm.setColor(color)
