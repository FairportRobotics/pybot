import magicbot
import components


class DriveForward(magicbot.AutonomousStateMachine):
    MODE_NAME = "Drive Forward"
    DEFAULT = True

    @magicbot.timed_state(duration=3, first=True)
    def drive_forward(self):
        pass
