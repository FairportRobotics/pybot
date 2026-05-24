"""Robot physics simulation engine.

This module is loaded automatically by ``robotpy sim`` when present in the
project root.  It is *not* used on the real robot.

Because the standalone ``phoenix6`` pip package's C-level simulation thread
crashes under pyfrc's HAL-sim, all hardware objects are replaced in sim by
lightweight Python stubs (``components/sim_devices.py``) gated on
``wpilib.RobotBase.isSimulation()``.

This physics engine reads ``motor_voltage`` directly from those stubs,
integrates it into velocity/position, and pushes results back into the
stubs so the rest of the codebase sees plausible sensor values.

Physics model
-------------
A simple linear motor model is used::

    velocity (rot/s) = voltage (V) / kV (V / (rot/s))

All modules are assumed to face forward in simulation.  No inertia or
slip is modelled — the goal is functional sim, not accurate dynamics.
"""

from __future__ import annotations

import math

from pyfrc.physics.core import PhysicsInterface
from wpimath.kinematics import ChassisSpeeds

import constants

# Number of intake motor rotations before a simulated game piece is captured.
# Represents the distance the intake must travel to pull a piece fully in.
_BEAM_BREAK_TRIGGER_ROT: float = 5.0


class PhysicsEngine:
    """pyfrc physics engine for the swerve drive robot.

    pyfrc creates one instance and calls:
    * ``__init__``     once at simulation start.
    * ``update_sim``   once every ~20 ms.
    """

    def __init__(self, physics_controller: PhysicsInterface, robot: object) -> None:
        self._physics = physics_controller

        swerve = robot.swerve_drive  # type: ignore[attr-defined]
        self._drive_stubs = [m._drive for m in swerve._modules]
        self._steer_stubs = [m._steer for m in swerve._modules]
        self._gyro_stub = swerve._gyro

        self._intake_stub = robot.intake._motor  # type: ignore[attr-defined]
        # Beam break stub — physics drives this to simulate game piece capture.
        self._beam_break_stub = robot.intake._beam_break  # type: ignore[attr-defined]
        self._shooter_stub = robot.shooter._motor  # type: ignore[attr-defined]

        # Accumulated heading (radians, CCW positive).
        self._heading_rad: float = 0.0
        # Integrated intake position used to trigger simulated beam break.
        self._intake_pos_rot: float = 0.0

    def update_sim(self, now: float, tm_diff: float) -> None:  # noqa: ARG002
        """Advance simulation by one time step.

        Parameters
        ----------
        now:
            Current wall-clock simulation time (seconds) — unused.
        tm_diff:
            Elapsed time since the previous call (seconds).
        """
        dt = tm_diff

        # --- Drive motors: voltage → velocity → position ---
        # Each module also contributes to chassis velocity weighted by its
        # simulated steer angle so that strafing shows correctly on the field.
        vx_total = 0.0
        vy_total = 0.0
        omega_num = 0.0  # numerator for omega estimate

        for i, (drive_stub, steer_stub) in enumerate(
            zip(self._drive_stubs, self._steer_stubs, strict=False)
        ):
            velocity_rps = drive_stub.motor_voltage / max(constants.DRIVE_KV, 1e-6)
            drive_stub._velocity_rps = velocity_rps
            drive_stub._position_rot += velocity_rps * dt

            velocity_mps = (
                velocity_rps / constants.DRIVE_GEAR_RATIO * constants.WHEEL_CIRCUMFERENCE_M
            )

            # Read the steer angle from the CANcoder stub (set by PositionVoltage).
            steer_angle_rad = steer_stub._position_rot * 2 * math.pi
            steer_stub._velocity_rps = 0.0  # stub: steer reaches target instantly

            vx_total += velocity_mps * math.cos(steer_angle_rad)
            vy_total += velocity_mps * math.sin(steer_angle_rad)

            # FL and FR modules contribute to omega (differential drive approx).
            half_tw = constants.TRACK_WIDTH_M / 2.0
            if i == 0:  # Front-Left
                omega_num += velocity_mps / half_tw
            elif i == 1:  # Front-Right
                omega_num -= velocity_mps / half_tw

        vx_mps = vx_total / len(self._drive_stubs)
        vy_mps = vy_total / len(self._drive_stubs)
        omega_rad_s = omega_num / 2.0  # average of FL and FR contributions

        # --- Intake motor ---
        intake_rps = self._intake_stub.motor_voltage / max(constants.INTAKE_KV, 1e-6)
        self._intake_stub._velocity_rps = intake_rps
        self._intake_stub._position_rot += intake_rps * dt
        self._intake_pos_rot += abs(intake_rps) * dt

        # Simulate beam break: after the intake has turned enough rotations
        # to pull a game piece in, break the beam.  Reset after the intake
        # stops so the next piece can be simulated.
        if intake_rps > 1.0 and self._intake_pos_rot >= _BEAM_BREAK_TRIGGER_ROT:
            self._beam_break_stub._value = False  # beam broken = piece detected
        elif intake_rps <= 0.0:
            # Intake stopped or reversed — piece was fired / ejected.
            self._beam_break_stub._value = True  # beam clear
            self._intake_pos_rot = 0.0

        # --- Shooter motor ---
        shooter_rps = self._shooter_stub.motor_voltage / max(constants.SHOOTER_KV, 1e-6)
        self._shooter_stub._velocity_rps = shooter_rps
        self._shooter_stub._position_rot += shooter_rps * dt

        # --- Integrate heading and push to gyro stub ---
        self._heading_rad += omega_rad_s * dt
        self._gyro_stub._yaw_deg = math.degrees(self._heading_rad)

        # --- Update pyfrc field display ---
        chassis_speeds = ChassisSpeeds(vx_mps, vy_mps, omega_rad_s)
        self._physics.drive(chassis_speeds, dt)
