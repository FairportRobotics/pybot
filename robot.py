import commands2
from robotcontainer import RobotContainer

"""
# AdvantageKit imports
from phoenix5_logger import (
    Logger,
    WPILOGWriter,
    WPILOGReader,
    NT4Publisher,
    LogFileUtil
)
"""


class Robot(commands2.TimedCommandRobot):
    def __init__(self):
        super().__init__()
        """
        # Log metadata
        Logger.recordMetadata("ProjectName", "MyProject")

        if wpilib.RobotBase.isReal():
            Logger.addDataReceiver(WPILOGWriter())  # Log to a USB stick
            Logger.addDataReceiver(NT4Publisher())  # NetworkTables
            PowerDistribution(1, PowerDistribution_ModuleType.kCTRE)  # Enable PDP logging
        else:
            self.setUseTiming(False)
            log_path = LogFileUtil.findReplayLog()
            Logger.setReplaySource(WPILOGReader(log_path))
            Logger.addDataReceiver(WPILOGWriter(LogFileUtil.addPathSuffix(log_path, "_sim")))

        Logger.start()
        """
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
        self.autonomous_command = None  # Not using autonomous anymore
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
