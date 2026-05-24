"""Autonomous mode: Do Nothing.

This is the **default** autonomous routine.  The robot stays in place for
the entire autonomous period.  It is a safe fallback — if we are not sure
where the robot is on the field, or if something breaks, doing nothing is
better than driving into a wall.

How MagicBot autonomous modes work
------------------------------------
Every Python file in the ``autonomous/`` folder is automatically discovered
by MagicBot and shown in the Driver Station's autonomous selector drop-down.
The display name comes from the ``MODE_NAME`` class variable.
``DEFAULT`` = True makes this the pre-selected choice when the DS connects.

The ``AutonomousStateMachine`` base class handles the state-machine loop.
Each ``@state`` method is one step.  ``first=True`` marks the entry point.
"""

from magicbot.state_machine import AutonomousStateMachine, state

from components.swerve_drive import SwerveDrive


class DoNothing(AutonomousStateMachine):
    """Autonomous routine that keeps the robot stationary."""

    # --- Driver Station selector ---
    MODE_NAME = "Do Nothing"  # Name shown in the DS autonomous chooser
    DEFAULT = True  # Pre-selected when the DS first connects

    # MagicBot injects the shared SwerveDrive component automatically.
    swerve_drive: SwerveDrive

    @state(first=True)
    def idle(self) -> None:
        """Stay still for the whole autonomous period.

        Calling ``swerve_drive.drive(0, 0, 0)`` every loop explicitly
        commands zero velocity, which keeps the wheel angles locked straight
        and the brakes engaged.  We never call ``self.next_state()`` so the
        robot stays here until autonomous ends.
        """
        self.swerve_drive.drive(vx=0.0, vy=0.0, omega=0.0)
