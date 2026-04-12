import wpilib
import wpimath.units as units
from wpilib.simulation import DCMotorSim
from wpimath.system.plant import DCMotor, LinearSystemId
from magicbot import feedback
import rev
from .base import MotorComponent


class NeoMotor(MotorComponent):
    """
    REV NEO brushless motor driven by a SparkMax controller.

    Supports both physical hardware and WPILib simulation transparently.

    Injected attributes (set in robot.py createObjects):
        can_id          (int)   - CAN bus ID of the SparkMax
        inverted        (bool)  - True to invert motor direction (default False)
        gear_ratio      (float) - Rotor rotations per output shaft rotation (default 1.0)
        moi             (float) - Moment of inertia in kg*m² (default 0.001)
        stall_limit     (int)   - Stall current limit in amps (default 40)
        free_limit      (int)   - Free speed current limit in amps (default 40)
        ramp_rate       (float) - Seconds from 0 to full throttle (default 0.1)
    """

    # --- Injected by MagicBot from createObjects() ---
    can_id:      int
    inverted:    bool  = False
    gear_ratio:  float = 1.0
    moi:         float = 0.001   # kg*m² — increase for heavier mechanisms
    stall_limit: int   = 40      # amps
    free_limit:  int   = 40      # amps
    ramp_rate:   float = 0.1     # seconds 0 -> full output

    # Internal sim constants
    _LOOP_PERIOD_S = 0.02        # 20 ms standard FRC loop
    _NOMINAL_VOLTAGE = 12.0      # volts

    def setup(self) -> None:
        """
        Called by MagicBot after injection. Initializes hardware or sim
        depending on runtime environment.
        """
        self._desired_output = 0.0
        self._is_simulation = wpilib.RobotBase.isSimulation()

        if self._is_simulation:
            self._setup_sim()
        else:
            self._setup_hardware()

    # -------------------------------------------------------------------------
    # Hardware setup
    # -------------------------------------------------------------------------

    def _setup_hardware(self) -> None:
        """Configure the physical SparkMax and NEO."""
        self._motor = rev.SparkMax(
            self.can_id,
            rev.SparkMax.MotorType.kBrushless
        )

        # Always restore defaults first to ensure a clean config

        self._motor.setInverted(self.inverted)
        #self._motor.setIdleMode(rev.SparkMax.IdleMode.kBrake)
        #self._motor.setOpenLoopRampRate(self.ramp_rate)

        # Current limits protect the motor and breakers
        #self._motor.setSmartCurrentLimit(
        #    self.stall_limit,
        #    self.free_limit
        #)

        self._encoder = self._motor.getEncoder()

    # -------------------------------------------------------------------------
    # Simulation setup
    # -------------------------------------------------------------------------

    def _setup_sim(self) -> None:
        """
        Set up a DCMotorSim plant to model the NEO's physics.

        DCMotorSim integrates motor voltage -> angular velocity -> position
        each loop, giving realistic encoder feedback without real hardware.
        """
        gearbox = DCMotor.NEO(1)
        plant = LinearSystemId.DCMotorSystem(gearbox, J=self.moi, gearing=self.gear_ratio)
        self._motor_sim = DCMotorSim(plant, gearbox)

        # Simulated encoder state — we maintain these ourselves in sim
        self._sim_position_rot = 0.0   # rotations
        self._sim_velocity_rpm = 0.0   # RPM (matches SparkMax encoder units)

    # -------------------------------------------------------------------------
    # MotorComponent interface
    # -------------------------------------------------------------------------

    def set_output(self, output: float) -> None:
        """
        Set motor duty cycle output.

        Args:
            output: Clamped to [-1.0, 1.0]. Positive = forward.
        """
        self._desired_output = max(-1.0, min(1.0, output))

    @feedback
    def get_velocity(self) -> float:
        """
        Returns:
            Motor velocity in RPM (output shaft, after gear ratio).
            Matches SparkMax encoder units for drop-in sim/real parity.
        """
        if self._is_simulation:
            return self._sim_velocity_rpm
        return self._encoder.getVelocity()

    @feedback
    def get_position(self) -> float:
        """
        Returns:
            Motor position in rotations (output shaft, after gear ratio).
        """
        if self._is_simulation:
            return self._sim_position_rot
        return self._encoder.getPosition()

    @feedback
    def get_output(self) -> float:
        """Returns the current commanded duty cycle output [-1.0, 1.0]."""
        return self._desired_output

    @feedback
    def get_current_amps(self) -> float:
        """
        Returns:
            Stator current draw in amps.
            In sim, derived from the DCMotorSim plant model.
        """
        if self._is_simulation:
            return abs(self._motor_sim.getCurrentDraw())
        return self._motor.getOutputCurrent()

    # -------------------------------------------------------------------------
    # execute() — called every loop by MagicBot
    # -------------------------------------------------------------------------

    def execute(self) -> None:
        """
        Apply the desired output to hardware or advance the sim plant.
        MagicBot calls this automatically every 20 ms.
        """
        if self._is_simulation:
            self._execute_sim()
        else:
            self._execute_hardware()

    def _execute_hardware(self) -> None:
        """Write the desired output to the physical SparkMax."""
        self._motor.set(self._desired_output)

    def _execute_sim(self) -> None:
        """
        Advance the DCMotorSim plant by one loop period.

        Flow:
            desired_output (duty cycle)
                -> input voltage
                -> DCMotorSim integrates physics
                -> angular velocity / position
                -> stored as sim encoder state
        """
        # Convert duty cycle to voltage
        input_voltage = self._desired_output * self._NOMINAL_VOLTAGE

        # Clamp to what the battery can actually supply
        input_voltage = max(
            -self._NOMINAL_VOLTAGE,
            min(self._NOMINAL_VOLTAGE, input_voltage)
        )

        # Step the physics simulation forward by one loop period
        self._motor_sim.setInputVoltage(input_voltage)
        self._motor_sim.update(self._LOOP_PERIOD_S)

        # Cache results in SparkMax-compatible units so get_velocity()
        # and get_position() behave identically in sim and on hardware.

        # DCMotorSim returns rad/s -> convert to RPM
        rad_per_s = self._motor_sim.getAngularVelocity()
        self._sim_velocity_rpm = units.radiansPerSecondToRotationsPerMinute(
            rad_per_s
        )

        # DCMotorSim returns radians -> convert to rotations
        radians = self._motor_sim.getAngularPosition()
        self._sim_position_rot = units.radiansToRotations(radians)