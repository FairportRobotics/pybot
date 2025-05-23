from commands2.button import CommandXboxController
from constants import UnnecicaryFancyEnumsForShooting
from commands.agitator_set_command import AgitatorSetCommand
from commands.feeder_command import FeederCommand
from commands.intake_assembely_toggle_command import IntakeAssembelyToggleCommand
from commands.intake_motor_command import IntakeMotorCommand
from commands.shooter_set_command import ShooterSetCommand
from commands.swerve_drive_command import SwerveDriveCommand
from commands.orientation_toggle_command import OrientationToggleCommand
from subsystems.agitator_subsystem import AgitatorSubsystem
from subsystems.feeder_subsystem import FeederSubsystem
from subsystems.gyro_subsystem import GyroSubsystem
from subsystems.intake_subsystem import IntakeSubsystem
from subsystems.shooter_subsystem import ShooterSubsystem
from subsystems.swerve_drive_subsystem import SwerveDriveSubsystem


class RobotContainer:
    def __init__(self):
        # Subsystems
        self.gyroSubsystem = GyroSubsystem("GyroSubsystem")
        self.agitatorSubsystem = AgitatorSubsystem()
        self.swerveDriveSubsystem = SwerveDriveSubsystem(self.gyroSubsystem)
        self.feederSubsystem = FeederSubsystem()
        self.intakeSubsystem = IntakeSubsystem()
        self.shooterSubsystem = ShooterSubsystem()

        # Controller
        self.controller = CommandXboxController()

        # Default command for swerve drive
        SwerveDriveCommand(self.swerveDriveSubsystem, self.gyroSubsystem)

        # Configure button bindings
        self.configureButtonBindings()

    def configureButtonBindings(self):
        self.controller.rightTrigger().whileTrue(
            IntakeMotorCommand(self.intakeSubsystem, 0.5)
        )
        self.controller.rightBumper().onTrue(
            ShooterSetCommand(
                self.shooterSubsystem, UnnecicaryFancyEnumsForShooting.fastShoot
            )
        )
        self.controller.leftTrigger().whileTrue(
            IntakeMotorCommand(self.intakeSubsystem, -0.5)
        )
        self.controller.leftBumper().whileTrue(FeederCommand(self.feederSubsystem, 0.4))
        self.controller.povUp().onTrue(
            IntakeAssembelyToggleCommand(self.intakeSubsystem)
        )
        self.controller.y().whileTrue(AgitatorSetCommand(self.agitatorSubsystem, 0.4))
        self.controller.b().whileTrue(AgitatorSetCommand(self.agitatorSubsystem, -0.4))
        self.controller.x().onTrue(
            ShooterSetCommand(
                self.shooterSubsystem, UnnecicaryFancyEnumsForShooting.slowShoot
            )
        )
        self.controller.leftStick().onTrue(
            OrientationToggleCommand(self.swerveDriveSubsystem)
        )
