import commands2
from robotcontainer import RobotContainer


class Robot(commands2.TimedCommandRobot):
    def robotInit(self):
        self.container = RobotContainer()

    def teleopInit(self):
        self.container.teleopInit()

    def teleopPeriodic(self):
        commands2.CommandScheduler.getInstance().run()

    def autonomousInit(self):
        auto_command = self.container.getAutonomousCommand()
        if auto_command:
            auto_command.schedule()

    def autonomousExit(self):
        if self.container.getAutonomousCommand():
            self.container.getAutonomousCommand().cancel()
