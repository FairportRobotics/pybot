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
        self.system = plant.LinearSystemId.identifyDrivetrainSystem(1.98, 0.2, 1.5, 0.3)

        self.drivesim = wpisim.DifferentialDrivetrainSim(
            self.system,
            inchesToMeters(
                constants.Robot.DISTANCE_BETWEEN_WHEELS_INCHES
            ),  # Track width (meters)
            plant.DCMotor.CIM(2),  # 2 CIM motors per side
            constants.Robot.GEAR_RATIO,  # Gearing (Toughbox Mini)
            inchesToMeters(
                constants.Robot.WHEEL_RADIUS_INCHES
            ),  # Wheel radius (meters)
        )

        # 2. Get Simulation Handles from your Robot's Talon SRX objects
        self.l_talon_sim = robot.drivetrain.left_motor.getSimCollection()
        self.r_talon_sim = robot.drivetrain.right_motor.getSimCollection()

    def update_sim(self, now: float, tm_diff: float) -> None:
        # 3. Pull voltages from the motor controllers and apply to physics
        self.drivesim.setInputs(
            self.l_talon_sim.getMotorOutputLeadVoltage(),
            self.r_talon_sim.getMotorOutputLeadVoltage(),
        )

        # 4. Advance the physics world
        self.drivesim.update(tm_diff)

        # 5. Convert distance (meters) to Talon ticks (4096 per rotation)
        # Formula: (Distance / Wheel Circumference) * Gear Ratio * 4096
        counts_per_m = (
            (1 / (2 * math.pi * inchesToMeters(constants.Robot.WHEEL_RADIUS_INCHES)))
            * constants.Robot.GEAR_RATIO
            * constants.Robot.ENCODER_TICKS_PER_ROTATION
        )

        # Update Talon Positions (Raw Ticks)
        self.l_talon_sim.setQuadratureRawPosition(
            int(self.drivesim.getLeftPosition() * counts_per_m)
        )
        self.r_talon_sim.setQuadratureRawPosition(
            int(self.drivesim.getRightPosition() * counts_per_m)
        )

        # Update Talon Velocities (Ticks per 100ms)
        self.l_talon_sim.setQuadratureVelocity(
            int(self.drivesim.getLeftVelocity() * counts_per_m / 10)
        )
        self.r_talon_sim.setQuadratureVelocity(
            int(self.drivesim.getRightVelocity() * counts_per_m / 10)
        )

        # 6. Update Field visualization
        # self.physics_controller.field.setRobotPose(self.drivesim.getPose())
