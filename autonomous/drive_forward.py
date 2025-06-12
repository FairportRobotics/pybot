import components
import magicbot


class DriveForward(magicbot.AutonomousStateMachine):
    MODE_NAME = "Drive Forward"
    DEFAULT = True

    drive: components.ArcadeDrive

    @magicbot.timed_state(duration=2.0, next_state="stop", first=True)
    def start(self):
        self.drive.go(0.9, 0)

    @magicbot.state()
    def stop(self):
        self.drive.stop()
        self.done()
