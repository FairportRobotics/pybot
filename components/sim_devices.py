"""Simulation stub devices for Phoenix 6 hardware.

When running under ``robotpy sim`` (pyfrc), the standalone ``phoenix6``
pip package's internal simulation thread crashes because it expects CTR's
own simulation framework rather than pyfrc's HAL-sim.

This module provides lightweight Python stub classes that implement the
same interface as the Phoenix 6 hardware objects (``TalonFX``,
``CANcoder``, ``Pigeon2``) so all component code runs unmodified in
simulation.  The stubs return zero/default values for all sensor reads
and silently accept all configuration and control calls.

Usage
-----
In each component ``__init__``, gate hardware creation with::

    import wpilib
    from components.sim_devices import SimTalonFX, SimCANcoder, SimPigeon2

    if wpilib.RobotBase.isSimulation():
        self._motor = SimTalonFX()
    else:
        self._motor = phoenix6.hardware.TalonFX(can_id, bus)

The real robot path is never changed; stubs only appear in simulation.
"""

from __future__ import annotations

# ---------------------------------------------------------------------------
# Shared helpers
# ---------------------------------------------------------------------------


class _SimSignal:
    """Mimics a Phoenix 6 ``StatusSignal`` — has a ``.value`` attribute."""

    def __init__(self, value: float = 0.0) -> None:
        self.value: float = value


class _SimConfigurator:
    """Accepts ``apply()`` calls silently."""

    def apply(self, _config: object) -> None:  # noqa: ANN001
        """No-op: configuration is ignored in simulation."""


# ---------------------------------------------------------------------------
# TalonFX stub
# ---------------------------------------------------------------------------


class SimTalonFX:
    """Stub replacement for ``phoenix6.hardware.TalonFX`` in simulation.

    All control requests are stored so ``physics.py`` can read commanded
    voltages for a basic kinematic simulation.
    """

    def __init__(self) -> None:
        self.configurator = _SimConfigurator()
        # Simulated sensor state — physics.py updates these each loop.
        self._velocity_rps: float = 0.0
        self._position_rot: float = 0.0
        self._duty_cycle: float = 0.0
        self._supply_current: float = 0.0
        # Last commanded voltage (set by physics.py after reading control).
        self.motor_voltage: float = 0.0

    # --- Control ---

    def set_control(self, request: object) -> None:  # noqa: ANN001
        """Store the control request type for physics integration."""
        output = getattr(request, "output", None)
        velocity = getattr(request, "velocity", None)
        if isinstance(request, VoltageOut):
            # VoltageOut.output is already in volts.
            self.motor_voltage = float(output)  # type: ignore[arg-type]
            self._duty_cycle = self.motor_voltage / 12.0
        elif output is not None:
            # DutyCycleOut: output is a duty cycle fraction [-1, 1].
            self.motor_voltage = float(output) * 12.0
            self._duty_cycle = float(output)
        elif velocity is not None:
            # VelocityVoltage: approximate voltage from velocity setpoint and kV.
            from constants import DRIVE_KV  # local import to avoid circularity

            self.motor_voltage = float(velocity) * DRIVE_KV
        else:
            # NeutralOut or unknown — coast to zero.
            self.motor_voltage = 0.0
            self._duty_cycle = 0.0

    # --- Sensor reads ---

    def get_velocity(self) -> _SimSignal:
        """Return a signal with the current simulated velocity (rot/s)."""
        return _SimSignal(self._velocity_rps)

    def get_position(self) -> _SimSignal:
        """Return a signal with the current simulated position (rotations)."""
        return _SimSignal(self._position_rot)

    def get_duty_cycle(self) -> _SimSignal:
        """Return a signal with the last applied duty cycle (-1 to 1)."""
        return _SimSignal(self._duty_cycle)

    def get_supply_current(self) -> _SimSignal:
        """Return a signal with simulated supply current (A)."""
        return _SimSignal(self._supply_current)


# ---------------------------------------------------------------------------
# CANcoder stub
# ---------------------------------------------------------------------------


class SimCANcoder:
    """Stub replacement for ``phoenix6.hardware.CANcoder`` in simulation."""

    def __init__(self, device_id: int = 0) -> None:
        self.configurator = _SimConfigurator()
        self.device_id: int = device_id
        self._position_rot: float = 0.0

    def get_absolute_position(self) -> _SimSignal:
        """Return a signal with the simulated absolute position (rotations)."""
        return _SimSignal(self._position_rot)


# ---------------------------------------------------------------------------
# Control request stubs (match phoenix6.controls interface)
# ---------------------------------------------------------------------------


class VelocityVoltage:
    """Sim stub for ``phoenix6.controls.VelocityVoltage``."""

    def __init__(self, velocity: float, slot: int = 0, enable_foc: bool = False) -> None:  # noqa: ARG002
        self.velocity = velocity


class PositionVoltage:
    """Sim stub for ``phoenix6.controls.PositionVoltage``."""

    def __init__(self, position: float, slot: int = 0, enable_foc: bool = False) -> None:  # noqa: ARG002
        self.position = position


class DutyCycleOut:
    """Sim stub for ``phoenix6.controls.DutyCycleOut``."""

    def __init__(self, output: float) -> None:
        self.output = output


class VoltageOut:
    """Sim stub for ``phoenix6.controls.VoltageOut``."""

    def __init__(self, output: float, enable_foc: bool = False) -> None:  # noqa: ARG002
        self.output = output  # volts (not duty cycle — handled in set_control)


class NeutralOut:
    """Sim stub for ``phoenix6.controls.NeutralOut``."""


# ---------------------------------------------------------------------------
# Pigeon 2 stub
# ---------------------------------------------------------------------------


class SimPigeon2:
    """Stub replacement for ``phoenix6.hardware.Pigeon2`` in simulation."""

    def __init__(self) -> None:
        self.configurator = _SimConfigurator()
        self._yaw_deg: float = 0.0

    def set_yaw(self, yaw_deg: float) -> None:
        """Set the simulated yaw in degrees."""
        self._yaw_deg = yaw_deg

    def get_yaw(self) -> _SimSignal:
        """Return a signal with the current simulated yaw (degrees)."""
        return _SimSignal(self._yaw_deg)


# ---------------------------------------------------------------------------
# DigitalInput stub
# ---------------------------------------------------------------------------


class SimDigitalInput:
    """Stub replacement for ``wpilib.DigitalInput`` in simulation.

    Defaults to ``True`` (beam clear / no game piece).  Physics code can
    drive ``_value`` to ``False`` to simulate a game piece blocking the beam.
    """

    def __init__(self, channel: int = 0) -> None:  # noqa: ARG002
        self._value: bool = True  # True = beam clear, False = beam broken

    def get(self) -> bool:
        """Return the current simulated digital input state."""
        return self._value


# ---------------------------------------------------------------------------
# Servo stub (for Blinkin LED driver)
# ---------------------------------------------------------------------------


class SimServo:
    """Stub replacement for ``wpilib.Servo`` in simulation.

    Accepts ``set()`` calls silently so LED code compiles and runs in sim.
    """

    def __init__(self, channel: int = 0) -> None:  # noqa: ARG002
        self._value: float = 0.0

    def set(self, value: float) -> None:
        """Store the commanded servo position (0.0–1.0)."""
        self._value = max(0.0, min(1.0, value))
