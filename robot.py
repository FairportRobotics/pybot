import commands2
from robotcontainer import RobotContainer
import wpilib

# Mocked AdvantageKit-like imports (You would need real Python bindings or mocks)
# from advantagekit.logger import Logger, WPILOGReader, WPILOGWriter, NT4Publisher, LogFileUtil
import constants  # This should define CURRENT_MODE with values REAL, SIM, REPLAY


class Robot(wpilib.TimedRobot):
    def robotInit(self):
        self.autoChooser = wpilib.SendableChooser()
        self.m_robotContainer = RobotContainer()
        self.m_autonomousCommand = None

        self.MAIN_TAB = wpilib.shuffleboard.Shuffleboard.getTab("Main")
        self.MAIN_TAB.add(self.autoChooser)
        """
        logger = Logger.getInstance()

        if constants.CURRENT_MODE == constants.Mode.REAL:
            logger.addDataReceiver(WPILOGWriter("/media/sda1/"))
            logger.addDataReceiver(NT4Publisher())
        elif constants.CURRENT_MODE == constants.Mode.SIM:
            logger.addDataReceiver(WPILOGWriter(""))
            logger.addDataReceiver(NT4Publisher())
        elif constants.CURRENT_MODE == constants.Mode.REPLAY:
            self.setUseTiming(False)
            logPath = LogFileUtil.findReplayLog()
            logger.setReplaySource(WPILOGReader(logPath))
            logger.addDataReceiver(WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")))

        logger.start()
        """

    def robotPeriodic(self):
        commands2.CommandScheduler.getInstance().run()

    def disabledInit(self):
        pass  # Insert lighting or other behavior here

    def disabledPeriodic(self):
        pass

    def disabledExit(self):
        pass

    def autonomousInit(self):
        self.m_autonomousCommand = self.autoChooser.getSelected()
        if self.m_autonomousCommand:
            self.m_autonomousCommand.schedule()

    def autonomousPeriodic(self):
        pass

    def autonomousExit(self):
        pass

    def teleopInit(self):
        if self.m_autonomousCommand:
            self.m_autonomousCommand.cancel()
        self.m_robotContainer.getTeleopDriveCommand().schedule()

    def teleopPeriodic(self):
        pass  # Logger output or gamepiece tracking could go here

    def teleopExit(self):
        pass

    def testInit(self):
        commands2.CommandScheduler.getInstance().cancelAll()

    def testPeriodic(self):
        pass

    def testExit(self):
        pass
