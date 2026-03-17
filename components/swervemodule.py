import constants
import math
import wpimath.kinematics
from phoenix6.hardware import TalonFX, CANcoder
from phoenix6.controls import VelocityVoltage, PositionVoltage, NeutralOut
from wpimath.geometry import Rotation2d


class SwerveModule:
    drive_motor: TalonFX
    steer_motor: TalonFX
    cancoder: CANcoder

    def setup(self):
        """Called by MagicBot after injection; initialise hardware here."""
        # Control requests (reused to avoid allocation in the loop)
        self.drive_velocity_ctrl = VelocityVoltage(0).with_slot(0)
        self.steer_position_ctrl = PositionVoltage(0).with_slot(0)
        self.neutral = NeutralOut()

        # Desired state set by the drivetrain component
        self.desired_state = wpimath.kinematics.SwerveModuleState(0.0, Rotation2d(0.0))

    def execute(self):
        speed_mps = self.desired_state.speed
        angle_rot = self.desired_state.angle.rotations()  # rotations for TalonFX

        if abs(speed_mps) < 0.01:
            # Park: hold steer angle, stop drive
            self.drive_motor.set_control(self.neutral)
        else:
            # Drive: velocity in RPS (rotations per second at the mechanism)
            speed_rps = speed_mps / constants.METERS_PER_ROTATION
            self.drive_motor.set_control(
                self.drive_velocity_ctrl.with_velocity(speed_rps)
            )

        # Steer: position in rotations (continuous wrap handles ±180 flip)
        self.steer_motor.set_control(self.steer_position_ctrl.with_position(angle_rot))

    # ------------------------------------------------------------------
    # Public API called by SwerveDrive component
    # ------------------------------------------------------------------

    def set_desired_state(self, state: wpimath.kinematics.SwerveModuleState):
        """
        Set the desired velocity (m/s) and heading (Rotation2d).
        Optimization (shortest-path) is applied automatically via
        continuous wrap in the TalonFX closed-loop config.
        """
        # Optimise: flip drive direction if turning > 90° is needed
        self.desired_state = wpimath.kinematics.SwerveModuleState.optimize(
            state, self.get_steer_angle()
        )

    def get_state(self) -> wpimath.kinematics.SwerveModuleState:
        """Return the current measured state (velocity + angle)."""
        return wpimath.kinematics.SwerveModuleState(
            self.drive_motor.get_velocity().value * constants.METERS_PER_ROTATION,
            self.get_steer_angle(),
        )

    def get_position(self) -> wpimath.kinematics.SwerveModulePosition:
        """Return the current position (distance + angle) for odometry."""
        return wpimath.kinematics.SwerveModulePosition(
            self.drive_motor.get_position().value * constants.METERS_PER_ROTATION,
            self.get_steer_angle(),
        )

    def get_steer_angle(self) -> Rotation2d:
        """Return the current absolute steer angle."""
        return Rotation2d(self.cancoder.get_absolute_position().value * 2 * math.pi)

    def stop(self):
        self.drive_motor.set_control(self.neutral)
        self.steer_motor.set_control(self.neutral)
