from magicbot import AutonomousStateMachine, state


class DoNothing(AutonomousStateMachine):
    MODE_NAME = "Do Nothing"
    DEFAULT = True

    @state(first=True)
    def doing_nothing(self):
        pass
