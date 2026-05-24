"""State machine package.

Import as::

    import state_machine as sm

    class MyRobot(GenieRobot):
        scoring: sm.ScoringStateMachine
"""

from state_machine.scoring import ScoringStateMachine

__all__ = ["ScoringStateMachine"]
