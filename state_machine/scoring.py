"""Scoring state machine.

Coordinates the full game-piece scoring sequence as a MagicBot
``StateMachine``.  It bridges the ``Intake``, ``Shooter``, and
``XboxController`` components so that ``robot.py`` only needs to
call ``self.scoring.request_intake()`` and the state machine
handles the rest automatically.

State diagram
-------------

                    driver holds left trigger
                    ──────────────────────────►
  ┌──────────────┐                              ┌────────────────┐
  │    IDLE       │                              │   INTAKING     │
  │  (waiting)   │◄─────────────────────────────│  intake motor  │
  └──────────────┘   beam break trips OR         │  running       │
         │           trigger released            └───────┬────────┘
         │                                               │ beam break trips
         │                                               ▼
         │                                      ┌────────────────┐
         │                                      │   HOLDING      │
         │                                      │ piece in robot │
         │                                      │ flywheel spins │
         │                                      └───────┬────────┘
         │                                              │ right bumper pressed
         │                                              │ AND flywheel at speed
         │                                              ▼
         │                                      ┌────────────────┐
         │          sequence complete            │    FIRING      │
         └──────────────────────────────────────│ feed piece in  │
                                                └────────────────┘

States
------
``idle``
    Nothing is happening.  All mechanisms are stopped.
    Entry point: driver calls ``request_intake()`` via left trigger.

``intaking``
    The intake motor runs, pulling a game piece into the robot.
    The flywheel begins spinning up immediately so it is ready sooner.
    Exits when:
      * The beam break trips (piece captured) → ``holding``
      * The driver releases the trigger       → ``idle``

``holding``
    A game piece is inside the robot.  The intake stops.
    The flywheel continues spinning up to target speed.
    Exits when:
      * The driver presses the right bumper → ``firing`` (if at speed)

``firing``
    The intake runs briefly in reverse — this feeds the game piece
    into the spinning flywheel and launches it.
    Exits after ``FIRE_DURATION_S`` seconds → ``idle``

How to wire into robot.py
--------------------------
1.  Import at the top of ``robot.py``::

        import state_machine as sm

2.  Declare the component::

        class MyRobot(GenieRobot):
            scoring: sm.ScoringStateMachine

    MagicBot automatically injects ``intake``, ``shooter``, and
    ``driver_controller`` because the names match the robot's
    component/variable declarations.

3.  In ``teleopPeriodic()``::

        # Scoring state machine
        if ctrl.left_trigger > constants.TRIGGER_THRESHOLD:
            self.scoring.request_intake()
        if ctrl.right_bumper:
            self.scoring.request_fire()
"""

from __future__ import annotations

from magicbot.state_machine import StateMachine, state, timed_state

from components.intake import Intake
from components.shooter import Shooter

# How long the intake feeds the piece into the flywheel (seconds).
# Long enough to fully launch the game piece; short enough not to jam.
FIRE_DURATION_S: float = 0.5


class ScoringStateMachine(StateMachine):
    """MagicBot StateMachine — full game-piece scoring sequence.

    MagicBot injects the shared component instances automatically
    because the attribute names match the robot-level declarations.
    """

    # --- MagicBot-injected components ---
    intake: Intake
    shooter: Shooter

    # ---------------------------------------------------------------
    # Public API — called from robot.py teleopPeriodic()
    # ---------------------------------------------------------------

    def request_intake(self) -> None:
        """Signal that the driver wants to run the intake.

        Call this every loop while the left trigger is held.
        The state machine will start intaking if it is currently idle,
        or stay in its current state if already running.
        """
        # engage() starts the machine from idle; it is a no-op if already
        # running.  The actual state progression is managed internally.
        self.engage()

    def request_fire(self) -> None:
        """Signal that the driver wants to fire the game piece.

        Call this when the right bumper is pressed.  The shot is only
        taken if a game piece is held AND the flywheel is at speed.
        Calling this in any other state is a safe no-op.
        """
        if self.current_state == "holding" and self.shooter.is_at_speed():
            self.next_state("firing")

    # ---------------------------------------------------------------
    # States
    # ---------------------------------------------------------------

    @state(first=True)
    def idle(self) -> None:
        """Wait for the driver to request an action.

        Both mechanisms are stopped.  The state machine sits here
        until ``request_intake()`` is called (via ``engage()``).
        StateMachine.engage() will re-enter ``intaking`` from here.
        """
        self.shooter.stop()

    @state
    def intaking(self) -> None:
        """Run the intake and start spinning the flywheel.

        The flywheel begins spinning now so it reaches target speed
        by the time the game piece is captured — minimising the wait
        time in ``holding``.

        Exits to ``holding`` automatically when the beam break trips
        (game piece detected inside the robot).
        The driver must keep holding the trigger; if they release it
        ``engage()`` stops being called and the state machine idles.
        """
        self.intake.intake()
        self.shooter.spin_up()

        if self.intake.has_game_piece():
            self.next_state("holding")

    @state
    def holding(self) -> None:
        """Hold the game piece and wait for the flywheel to reach speed.

        The intake stops (the beam break interlock in ``Intake.execute()``
        will keep it stopped regardless).  The flywheel continues to
        spin up.  An LED or dashboard indicator should show the driver
        that a piece is held and whether the shot is ready.

        ``request_fire()`` transitions to ``firing`` once the flywheel
        is at speed.  The driver triggers this with the right bumper.
        """
        # Keep the shooter spinning while we wait.
        self.shooter.spin_up()

    @timed_state(duration=FIRE_DURATION_S, next_state="idle")
    def firing(self) -> None:
        """Feed the game piece into the flywheel and fire.

        The intake runs forward to push the held game piece into the
        spinning flywheel.  After ``FIRE_DURATION_S`` seconds the
        state machine returns to ``idle`` automatically and the flywheel
        coasts down.
        """
        self.intake.intake()
        self.shooter.spin_up()
