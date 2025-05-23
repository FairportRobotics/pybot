import math
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState
from ctre import WPI_TalonFX  # or WPI_TalonSRX depending on hardware

from constants import DriveConstants


class SwerveModule:
    def __init__(self, module_number: int, module_position):
        """
        :param module_number: index of the module (0-3)
        :param module_position: Translation2d for kinematics
        """
        self.module_number = module_number
        self.module_position = module_position

        # Motor IDs - define in your constants.py
        self.drive_motor = WPI_TalonFX(DriveConstants.kDriveMotorIDs[module_number])
        self.steer_motor = WPI_TalonFX(DriveConstants.kSteerMotorIDs[module_number])

        # Configure motors here (e.g. factory defaults, sensor phase, inversion)
        self.drive_motor.configFactoryDefault()
        self.steer_motor.configFactoryDefault()

        # Configure PID, sensor positions, etc. for steering motor
        # This depends on your hardware & sensors setup

        # Store desired state for telemetry/debugging
        self.last_desired_state = SwerveModuleState(0, Rotation2d())

    def get_absolute_steer_angle(self) -> Rotation2d:
        """
        Get the current angle of the steering motor in Rotation2d.
        You will need to read your encoder here and convert to radians.

        This is hardware-dependent; example:
        """
        # Assuming encoder ticks mapped to 0-360 degrees or 0-2pi radians:
        ticks = self.steer_motor.getSelectedSensorPosition()
        angle_rad = (
            (ticks / DriveConstants.kSteerEncoderTicksPerRevolution) * 2 * math.pi
        )
        # Wrap angle between 0 and 2pi
        angle_rad %= 2 * math.pi
        return Rotation2d(angle_rad)

    def get_drive_velocity(self) -> float:
        """
        Get current velocity of drive motor in meters per second.
        Needs conversion based on encoder velocity units.

        Example conversion (hardware dependent):
        """
        sensor_velocity = self.drive_motor.getSelectedSensorVelocity()
        # Convert ticks per 100ms to meters per second:
        meters_per_sec = (
            sensor_velocity * DriveConstants.kDriveEncoderDistancePerPulse * 10
        )
        return meters_per_sec

    def get_state(self) -> SwerveModuleState:
        """
        Returns current speed and angle of this swerve module.
        """
        return SwerveModuleState(
            self.get_drive_velocity(), self.get_absolute_steer_angle()
        )

    def set_desired_state(
        self, desired_state: SwerveModuleState, open_loop: bool = False
    ):
        """
        Sets the desired state for the module.

        :param desired_state: target speed and angle
        :param open_loop: if True, sets motor outputs directly (no PID)
        """
        # Optimize state to minimize rotation
        current_angle = self.get_absolute_steer_angle()
        optimized_state = SwerveModuleState.optimize(desired_state, current_angle)

        # Save for telemetry/debugging
        self.last_desired_state = optimized_state

        # Drive motor output
        if open_loop:
            percent_output = (
                optimized_state.speed / DriveConstants.kMaxSpeedMetersPerSecond
            )
            self.drive_motor.set(percent_output)
        else:
            # Use PID controller or velocity control mode to control velocity here
            # Example using velocity setpoint (ticks per 100ms):
            velocity_ticks_per_100ms = (
                optimized_state.speed
                / DriveConstants.kDriveEncoderDistancePerPulse
                / 10
            )
            self.drive_motor.set(velocity_ticks_per_100ms)

        # Steer motor control - convert target angle to sensor units and command motor
        target_angle_rad = optimized_state.angle.radians()
        target_ticks = (
            target_angle_rad / (2 * math.pi)
        ) * DriveConstants.kSteerEncoderTicksPerRevolution
        current_ticks = self.steer_motor.getSelectedSensorPosition()

        # Calculate shortest path error for steering
        error_ticks = target_ticks - current_ticks
        # Wrap error to [-ticks_per_rev/2, ticks_per_rev/2]
        half_rev = DriveConstants.kSteerEncoderTicksPerRevolution / 2
        if error_ticks > half_rev:
            error_ticks -= DriveConstants.kSteerEncoderTicksPerRevolution
        elif error_ticks < -half_rev:
            error_ticks += DriveConstants.kSteerEncoderTicksPerRevolution

        # Simple P control for steering as example
        kP = DriveConstants.kSteerKP
        steer_output = kP * error_ticks
        self.steer_motor.set(steer_output)

    def stop(self):
        """
        Stops both drive and steer motors.
        """
        self.drive_motor.set(0)
        self.steer_motor.set(0)
