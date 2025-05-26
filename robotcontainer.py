import commands
import constants
import subsystems


class RobotContainer:
    def __init__(self):
        self.drivetrain_type = constants.DRIVETRAIN_TYPE.lower()
        if self.drivetrain_type not in ["tank", "swerve"]:
            raise ValueError(f"Invalid drivetrain type: {self.drivetrain_type}")
        # Subsystems
        self.climber = subsystems.Climber()
        self.controller = subsystems.Controller(
            controller_type=constants.CONTROLLER_TYPE
        )
        if self.drivetrain_type == "tank":
            self.drive = subsystems.TankDrive()
        elif self.drivetrain_type == "swerve":
            self.drive = subsystems.SwerveDrive()
        self.elevator = subsystems.Elevator()
        self.feeder = subsystems.Feeder()
        self.intake = subsystems.Intake()
        self.network_tables = subsystems.NetworkTables()
        self.shooter = subsystems.Shooter()

        # Configure button bindings
        self.configureButtonBindings()

    def configureButtonBindings(self):
        pass
