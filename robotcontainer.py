import commands
import constants
import subsystems


class RobotContainer:
    def __init__(self):
        # Subsystems
        self.controller = subsystems.Controller(
            controller_type=constants.CONTROLLER_TYPE
        )
        if constants.DRIVETRAIN_TYPE == "tank":
            self.driveSubsystem = subsystems.TankDrive()
        elif constants.DRIVETRAIN_TYPE == "swerve":
            self.drive = subsystems.SwerveDrive()
        # self.elevator = subsystems.Elevator()
        # self.feeder = subsystems.Feeder()
        # self.intake = subsystems.Intake()
        self.network_tables = subsystems.NetworkTables()
        # self.shooter = subsystems.Shooter()

        # Configure button bindings
        self.configureButtonBindings()

    def configureButtonBindings(self):
        pass
