import commands
import subsystems


class RobotContainer:
    def __init__(self):
        self.controller = subsystems.Controller("Xbox")
        self.drive_subsystem = subsystems.Drive()
        self.drive_command = commands.Drive(self.drive_subsystem, self.controller)
        self.configure_bindings()
        self.drive_subsystem.setDefaultCommand(self.drive_command)

    def configure_bindings(self):
        # Bind buttons to commands here if needed
        pass

    def teleopInit(self):
        # Reset sensors or state here if needed
        pass

    def getAutonomousCommand(self):
        return None  # Replace with autonomous command if needed
