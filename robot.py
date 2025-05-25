import commands2
from robotcontainer import RobotContainer


class Robot(commands2.TimedCommandRobot):
    def __init__(self):
        super().__init__()
        # Instantiate the robot container
        self.container = RobotContainer()
        self.autonomous_command = None

    def robotPeriodic(self):
        commands2.CommandScheduler.getInstance().run()

    def disabledInit(self):
        pass

    def disabledPeriodic(self):
        pass

    def autonomousInit(self):
        if self.autonomous_command:
            self.autonomous_command.schedule()

    def autonomousPeriodic(self):
        pass

    def teleopInit(self):
        if self.autonomous_command:
            self.autonomous_command.cancel()

    def teleopPeriodic(self):
        pass

    def testInit(self):
        commands2.CommandScheduler.getInstance().cancelAll()

    def testPeriodic(self):
        pass

    def simulationInit(self):
        pass

    def simulationPeriodic(self):
        pass
