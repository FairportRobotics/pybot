import components
import constants
from magicbot import MagicRobot, feedback
import wpilib.deployinfo


class MyRobot(MagicRobot):
    accelerometer: components.RoboRioAccelerometer
    controller: components.XboxController

    @property
    def current_state(self):
        return self._current_state

    @current_state.setter
    def current_state(self, new_state: str):
        self._current_state = new_state

    def createObjects(self):
        self.current_state = "Initializing"

        """Create motors and stuff here"""
        # Controller stuff here
        self.controller_correct_for_deadband = constants.CONTROLLER_CORRECT_FOR_DEADBAND
        self.controller_deadband = constants.CONTROLLER_DEADBAND
        self.controller_port = constants.CONTROLLER_PORT

        # Get info about the code deployed to the robot
        self.deploy_data = wpilib.deployinfo.getDeployData()
        # if no deployment data is found, create an empty dict to prevent errors
        if self.deploy_data is None:
            self.deploy_data = {}

    def autonomous(self):
        self.current_state = "Starting autonomous routine"

    def teleopInit(self):
        self.current_state = "Controlled by humans"
        self.accelerometer.reset()

    def teleopPeriodic(self):
        self.controller.capture_button_presses()
        left_x, left_y, right_x, right_y = self.controller.get_joysticks()

    def disabledInit(self):
        self.current_state = "Awaiting your command"
        self.accelerometer.reset()

    def disabledPeriodic(self):
        pass

    """
    Information for the dashaboard
    """

    @feedback
    def git_branch(self):
        return self.deploy_data.get("git-branch", "Unknown")

    @feedback
    def git_hash(self):
        return self.deploy_data.get("git-hash", "Unknown")

    @feedback
    def deploy_date(self):
        return self.deploy_data.get("deploy-date", "Unknown")

    @feedback
    def is_stationary(self):
        return self.accelerometer.stationary

    @feedback(key="current_state")
    def get_current_state(self):
        return self.current_state
