from magicbot import AutonomousStateMachine, timed_state, state


class DoNothing(AutonomousStateMachine):
    MODE_NAME = "Do Nothing"
    DEFAULT = True

    @timed_state(duration=3, first=True, next_state="finish")
    def doing_nothing(self):
        print("Doing nothing")

    @state()
    def finish(self):
        print("Done")
