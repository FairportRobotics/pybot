import commands2
import magicbot


class GenieRobot(magicbot.MagicRobot):
    def robotPeriodic(self) -> None:
        commands2.CommandScheduler.getInstance().run()
        return super().robotPeriodic()

    def scheduleCommand(self, command: commands2.Command) -> None:
        """
        Schedule a command to run. This is a wrapper around
        :meth:`CommandScheduler.add` that allows you to
        schedule commands.

        :param command: The command to schedule
        """
        inst = commands2.CommandScheduler.getInstance()
        inst.schedule(command)

    def disabledPeriodic(self):
        # MagicBot's disabledPeriodic() runs all component execute() loops
        # even while disabled.  This is important for Limelight — the cameras
        # keep fusing AprilTag pose estimates into the pose estimator while
        # the robot sits on the field before the match starts, so the robot
        # already knows where it is the moment autonomous begins.
        super().disabledPeriodic()
