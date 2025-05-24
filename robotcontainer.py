import commands
import commands2
import constants
import subsystems
import wpilib


class RobotContainer:
    def __init__(self):
        self.driver_controller = commands2.button.CommandXboxController(
            constants.DRIVER_XBOX_CONTROLLER_ID
        )
        self.pneumatics_control_module = wpilib.PneumaticsControlModule(
            constants.PH_CAN_ID
        )

        self.drive_subsystem = subsystems.DriveSubsystem()
        self.arm_subsystem = subsystems.ArmSubsystem()
        self.artillery_subsystem = subsystems.ArtillerySubsystem()

        self.drive_command = commands.TankDriveCommand()

        self._configure_bindings()
        self._init_compressor()

        # Add chooser to Shuffleboard if needed
        wpilib.shuffleboard.Shuffleboard.getTab("Main").add(
            "Drive Mode", self.drive_command
        )

    def _configure_bindings(self):
        self.driver_controller.rightTrigger().whileTrue(
            commands.ArmIntakeCommand(commands.ArmIntakeDirection.IN)
        )
        self.driver_controller.rightBumper().whileTrue(
            commands.ArmIntakeCommand(commands.ArmIntakeDirection.OUT)
        )
        self.driver_controller.leftTrigger().onTrue(
            commands.ArmExtentionCommand(commands.ArmExtentionDirection.EXTEND)
        )
        self.driver_controller.leftBumper().onTrue(
            commands.ArmExtentionCommand(commands.ArmExtentionDirection.RETRACT)
        )
        self.driver_controller.y().onTrue(
            commands.ArtilleryCommand(commands.ArtilleryPower.FULL)
        )
        self.driver_controller.a().onTrue(
            commands.ArtilleryCommand(commands.ArtilleryPower.HALF)
        )
        self.driver_controller.b().onTrue(
            commands.ArtilleryCommand(commands.ArtilleryPower.NONE)
        )

    def _init_compressor(self):
        self.pneumatics_control_module.enableCompressorDigital()
        self.pcm_compressor = wpilib.Compressor(0, wpilib.PneumaticsModuleType.CTREPCM)
        self.pcm_compressor.enableDigital()

    def get_teleop_drive_command(self) -> commands2.Command:
        return self.drive_command
