import wpilib.simulation as wpisim
import wpimath.system.plant as plant
from wpimath.units import inchesToMeters
from pyfrc.physics.core import PhysicsInterface
import constants
import math


class PhysicsEngine:
    def __init__(self, physics_controller: PhysicsInterface, robot: "MyRobot"):
        self.physics_controller = physics_controller

        # 1. Define the Drivetrain Model
        # Identify system constants (kV, kA) or use a standard motor/gearbox combo
        self.system = plant.LinearSystemId.identifyDrivetrainSystem(
            4.5, 0.293, 3.0, 0.3
        )

        self.system = plant.LinearSystemId.drivetrainVelocitySystem(
            plant.DCMotor.CIM(2),  # 2 CIMs per side
            constants.Robot.MASS_IN_KG,  # robot mass in kg
            inchesToMeters(constants.Robot.WHEEL_RADIUS_IN_INCHES),  # wheel radius
            inchesToMeters(
                constants.Robot.DISTANCE_BETWEEN_WHEELS_IN_INCHES / 2
            ),  # track width / 2
            constants.Robot.GEAR_RATIO,  # gear ratio
            constants.Robot.MOMENT_OF_INERTIA,  # kg·m²
        )

        self.simulated_drivetrain = wpisim.DifferentialDrivetrainSim(
            self.system,
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
        self.l_talon_sim = robot.drivetrain.left_motor.getSimCollection()
        self.r_talon_sim = robot.drivetrain.right_motor.getSimCollection()

    def update_sim(self, now: float, tm_diff: float) -> None:
        # 3. Pull voltages from the motor controllers and apply to physics
        self.simulated_drivetrain.setInputs(
            self.l_talon_sim.getMotorOutputLeadVoltage(),
            self.r_talon_sim.getMotorOutputLeadVoltage(),
        )

        # 4. Advance the physics world
        self.simulated_drivetrain.update(tm_diff)

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
        self.l_talon_sim.setQuadratureRawPosition(
            int(self.simulated_drivetrain.getLeftPosition())# * counts_per_m)
        )
        self.r_talon_sim.setQuadratureRawPosition(
            int(self.simulated_drivetrain.getRightPosition())# * counts_per_m)
        )

        # Update Talon Velocities (Ticks per 100ms)
        self.l_talon_sim.setQuadratureVelocity(
            int(self.simulated_drivetrain.getLeftVelocity() * counts_per_m / 10)
        )
        self.r_talon_sim.setQuadratureVelocity(
            int(self.simulated_drivetrain.getRightVelocity() * counts_per_m / 10)
        )
