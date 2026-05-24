"""Autonomous mode: PathPlanner Auto.

Runs a full autonomous routine built in the PathPlanner GUI.

How it works
------------
1. Open the PathPlanner desktop app and point it at this project folder.
2. Draw one or more paths under ``deploy/pathplanner/paths/``.
3. Compose an auto routine under ``deploy/pathplanner/autos/`` — drag paths
   and event markers into the timeline.
4. Change ``AUTO_NAME`` below to match the ``.auto`` file name (without the
   ``.auto`` extension).
5. Deploy to the robot. Select "PathPlanner Auto" on the Driver Station.

Named commands (optional)
--------------------------
In the PathPlanner GUI you can drop event markers on a path that fire named
commands at specific points.  Register them in ``robot.py createObjects()``::

    from pathplannerlib.auto import NamedCommands
    import commands2
    NamedCommands.registerCommand(
        "intake",
        commands2.InstantCommand(lambda: self.intake.intake()),
    )

The marker names in the GUI must exactly match the strings passed here.

Alliance flipping
-----------------
``AutoBuilder.configure()`` in ``robot.py`` passes a ``should_flip_path``
lambda that returns ``True`` when on the red alliance.  PathPlanner will
mirror the path automatically — you only need to draw paths for blue.

Debugging
---------
PathPlanner publishes the active path to SmartDashboard under
``/PathPlanner/``.  Open Elastic or Shuffleboard to see the robot following
the path in real time.
"""

from __future__ import annotations

from magicbot.state_machine import AutonomousStateMachine, state
from pathplannerlib.auto import AutoBuilder

# ---------------------------------------------------------------------------
# Change this to match the name of your .auto file in the PathPlanner GUI.
# ---------------------------------------------------------------------------
AUTO_NAME: str = "Example Auto"


class PathPlannerAuto(AutonomousStateMachine):
    """Run a PathPlanner GUI auto via the GenieRobot command scheduler."""

    MODE_NAME = "PathPlanner Auto"
    DEFAULT = False

    def on_enable(self) -> None:
        """Build and schedule the PathPlanner command when auto starts.

        ``AutoBuilder.buildAuto()`` reads the named ``.auto`` file from
        ``deploy/pathplanner/autos/``, constructs the full command sequence
        (path following + named command events), and returns a single
        ``Command`` object.

        ``self.scheduleCommand()`` is provided by ``GenieRobot`` — it hands
        the command to the ``CommandScheduler``, which ticks it every loop
        inside ``robotPeriodic()``.
        """
        auto_command = AutoBuilder.buildAuto(AUTO_NAME)
        self.scheduleCommand(auto_command)  # type: ignore[attr-defined]

    @state(first=True)
    def running(self) -> None:
        """Wait while the CommandScheduler runs the PathPlanner command.

        This state does nothing — the actual path following happens inside
        ``CommandScheduler.run()`` (called by GenieRobot every loop).
        We stay in this state until autonomous ends.
        """
