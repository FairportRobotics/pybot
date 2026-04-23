import wpilib.simulation as wpisim
import wpimath.system.plant as plant
from wpimath.units import inchesToMeters
from pyfrc.physics.core import PhysicsInterface
import constants
import components
import math


class PhysicsEngine:
    def __init__(self, physics_controller: PhysicsInterface, robot: "MyRobot"):
        self.physics_controller = physics_controller
        self.tank_drive: components.TankDrive = robot.drivetrain
        self.navx: components.NavX = robot.gyro

        # 1. Define the Drivetrain Model
        # Identify system constants (kV, kA) or use a standard motor/gearbox combo
        self.plant = plant.LinearSystemId.identifyDrivetrainSystem(4.5, 0.293, 3.0, 0.3)
        """
        self.plant = plant.LinearSystemId.drivetrainVelocitySystem(
            plant.DCMotor.CIM(2),  # 2 CIMs per side
            constants.Robot.MASS_IN_KG,  # robot mass in kg
            inchesToMeters(constants.Robot.WHEEL_RADIUS_IN_INCHES),  # wheel radius
            inchesToMeters(
                constants.Robot.DISTANCE_BETWEEN_WHEELS_IN_INCHES / 2
            ),  # track width / 2
            constants.Robot.GEAR_RATIO,  # gear ratio
            constants.Robot.MOMENT_OF_INERTIA,  # kg·m²
        )
        """
        self.simulated_drivetrain = wpisim.DifferentialDrivetrainSim(
            self.plant,
            inchesToMeters(
                constants.Robot.DISTANCE_BETWEEN_WHEELS_IN_INCHES
            ),  # Track width (meters)
            plant.DCMotor.CIM(2),  # 2 CIM motors per side
            constants.Robot.GEAR_RATIO,  # Gearing (Toughbox Mini)
            inchesToMeters(
                constants.Robot.WHEEL_RADIUS_IN_INCHES
            ),  # Wheel radius (meters)
        )

        # 2. Get Simulation Handles from your Robot's Talon SRX objects
        self.simulated_left_motor = robot.drivetrain.left_motor.getSimCollection()
        self.simulated_right_motor = robot.drivetrain.right_motor.getSimCollection()

    def update_sim(self, now: float, time_difference: float) -> None:
        # 3. Pull voltages from the motor controllers and apply to physics
        self.simulated_drivetrain.setInputs(
            self.simulated_left_motor.getMotorOutputLeadVoltage(),
            self.simulated_right_motor.getMotorOutputLeadVoltage(),
        )

        # 4. Advance the physics world
        self.simulated_drivetrain.update(time_difference)
        """
        # 5. Convert distance (meters) to Talon ticks (4096 per rotation)
        # Formula: (Distance / Wheel Circumference) * Gear Ratio * 4096
        counts_per_m = (
            constants.Robot.ENCODER_TICKS_PER_ROTATION  # ticks/motor-rotation
            * constants.Robot.GEAR_RATIO  # motor-rotations/wheel-rotation
            / (
                2 * math.pi * inchesToMeters(constants.Robot.WHEEL_RADIUS_IN_INCHES)
            )  # wheel-rotations/meter
        )

        # Update Talon Positions (Raw Ticks)
        self.simulated_left_motor.setQuadratureRawPosition(
            int(self.simulated_drivetrain.getLeftPosition())# * counts_per_m)
        )
        self.simulated_right_motor.setQuadratureRawPosition(
            int(self.simulated_drivetrain.getRightPosition())# * counts_per_m)
        )

        # Update Talon Velocities (Ticks per 100ms)
        self.simulated_left_motor.setQuadratureVelocity(
            int(self.simulated_drivetrain.getLeftVelocity() * counts_per_m / 10)
        )
        self.simulated_right_motor.setQuadratureVelocity(
            int(self.simulated_drivetrain.getRightVelocity() * counts_per_m / 10)
        )
        """
        # Push simulated encoder positions back to the motors
        self.tank_drive.left_motor.getSimCollection().setQuadratureRawPosition(
            int(self._meters_to_ticks(self.simulated_drivetrain.getLeftPosition()))
        )
        self.tank_drive.right_motor.getSimCollection().setQuadratureRawPosition(
            int(self._meters_to_ticks(self.simulated_drivetrain.getRightPosition()))
        )
        self.tank_drive.left_motor.getSimCollection().setQuadratureVelocity(
            int(
                self._mps_to_ticks_per_100ms(
                    self.simulated_drivetrain.getLeftVelocity()
                )
            )
        )
        self.tank_drive.right_motor.getSimCollection().setQuadratureVelocity(
            int(
                self._mps_to_ticks_per_100ms(
                    self.simulated_drivetrain.getRightVelocity()
                )
            )
        )

        # Push simulated heading into the NavX sim registers
        self.navx.sim_set_angle(self.simulated_drivetrain.getHeading().degrees())
        self.navx.sim_set_rate(
            self.simulated_drivetrain.getHeading().degrees()
            / time_difference  # rough rate
        )

    def _meters_to_ticks(self, meters: float) -> float:
        wheel_rotations = meters / (
            inchesToMeters(constants.Robot.WHEEL_RADIUS_IN_INCHES) * math.pi
        )
        motor_rotations = wheel_rotations * constants.Robot.GEAR_RATIO
        return motor_rotations * constants.Robot.ENCODER_TICKS_PER_ROTATION

    def _mps_to_ticks_per_100ms(self, mps: float) -> float:
        return self._meters_to_ticks(mps) / 10
