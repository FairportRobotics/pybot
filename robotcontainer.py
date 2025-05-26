import commands
import constants
import subsystems


class RobotContainer:
    def __init__(self):
        # Subsystems
        self.controller = subsystems.Controller(
            controller_type=constants.CONTROLLER_TYPE
        )
        drivetrain_type = constants.DRIVETRAIN_TYPE.lower()
        if drivetrain_type == "tank":
            self.drive = subsystems.TankDrive()
        elif drivetrain_type == "swerve":
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
