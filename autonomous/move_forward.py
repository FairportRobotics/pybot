"""Autonomous mode: Move Forward.

The robot drives straight forward (toward the opposing alliance wall) for
``DRIVE_DURATION_S`` seconds, then stops and holds position for the
remainder of autonomous.

Tune ``constants.AUTO_DRIVE_SPEED_MPS`` to change how fast it moves.
Positive ``vx`` is forward in robot-relative coordinates (toward the
front bumper of the robot as oriented at the start of the match).

State machine
-------------
  start_driving  ──(after DRIVE_DURATION_S s)──►  stop
       │                                              │
       ▼                                              ▼
  drive forward                               hold position (0, 0, 0)

How MagicBot autonomous modes work
------------------------------------
Every Python file in the ``autonomous/`` folder is automatically discovered
by MagicBot and shown in the Driver Station's autonomous selector drop-down.
The display name comes from the ``MODE_NAME`` class variable.

``@timed_state`` is like ``@state`` but automatically advances to the next
state after the specified duration has elapsed.  No timer code needed!
"""

from magicbot.state_machine import AutonomousStateMachine, state, timed_state
from wpimath.kinematics import ChassisSpeeds

import constants
from components.swerve_drive import SwerveDrive

# ---------------------------------------------------------------------------
# Tuning — change these values to adjust behaviour without editing logic
# ---------------------------------------------------------------------------

# How long to drive forward (seconds).
DRIVE_DURATION_S: float = 3.0


class MoveForward(AutonomousStateMachine):
    """Autonomous routine: drive straight forward, then stop."""

    # --- Driver Station selector ---
    MODE_NAME = "Move Forward"  # Name shown in the DS autonomous chooser
    DEFAULT = False  # Not the default — operator must select this

    # MagicBot injects the shared SwerveDrive component automatically.
    swerve_drive: SwerveDrive

    @timed_state(duration=DRIVE_DURATION_S, next_state="stop", first=True)
    def start_driving(self) -> None:
        """Drive straight forward at AUTO_DRIVE_SPEED_MPS.

        Uses ``drive_chassis_speeds()`` with a robot-relative forward command
        so the heading-hold PID in SwerveDrive keeps the robot straight even
        if the gyro drifts slightly.  vx is forward; vy and omega are zero.
        """
        self.swerve_drive.drive_chassis_speeds(
            ChassisSpeeds(constants.AUTO_DRIVE_SPEED_MPS, 0.0, 0.0)
        )

    @state
    def stop(self) -> None:
        """Hold position for the rest of autonomous.

        Commanding zero velocity every loop keeps the wheel brake mode
        active and prevents the robot from drifting.  We never call
        ``self.next_state()`` so the robot stays here until auto ends.
        """
        self.swerve_drive.drive(vx=0.0, vy=0.0, omega=0.0)
