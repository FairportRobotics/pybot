import phoenix6
from magicbot import feedback
from .base import MotorComponent
import wpilib
from wpilib.simulation import DCMotorSim
from wpimath.system.plant import DCMotor, LinearSystemId


class KrakenMotor(MotorComponent):
    """TalonFX-based Kraken X60 component."""

    # MagicBot injects this from the component's config
    can_id: int

    def setup(self) -> None:
        self._is_simulation = wpilib.RobotBase.isSimulation()
        self._motor = phoenix6.hardware.TalonFX(self.can_id)
        cfg = phoenix6.configs.TalonFXConfiguration()
        # configure current limits, ramp rates, etc.
        self._motor.configurator.apply(cfg)
        self._output = 0.0

        if self._is_simulation:
            self._sim_state = self._motor.sim_state
            gearbox = DCMotor.krakenX60(1)
            plant = LinearSystemId.DCMotorSystem(gearbox, J=0.001, gearing=1.0)
            self._motor_sim = DCMotorSim(plant, gearbox)

    def set_output(self, output: float) -> None:
        self._output = max(-1.0, min(1.0, output))

    @feedback
    def get_velocity(self) -> float:
        return self._motor.get_velocity().value

    @feedback
    def get_position(self) -> float:
        return self._motor.get_position().value

    def execute(self) -> None:
        self._motor.set_control(phoenix6.controls.DutyCycleOut(self._output))
        if self._is_simulation:
            self._update_sim()

    def _update_sim(self) -> None:
        self._sim_state.set_supply_voltage(12.0)
        self._motor_sim.setInputVoltage(self._sim_state.motor_voltage)
        self._motor_sim.update(0.02)  # 20ms loop period
        self._sim_state.set_raw_rotor_position(self._motor_sim.getAngularPosition())
        self._sim_state.set_rotor_velocity(self._motor_sim.getAngularVelocity())
