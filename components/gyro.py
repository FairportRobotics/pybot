import navx
from wpimath.geometry import Rotation2d
from wpilib import RobotBase
from wpilib.simulation import SimDeviceSim
from magicbot import feedback


class NavX:
    """MagicBot component wrapping the navX-MXP/navX-micro gyro."""

    def setup(self) -> None:
        self._navx = navx.AHRS.create_spi()

        self._sim_angle = 0.0  # degrees, cumulative

        if RobotBase.isSimulation():
            # The navX registers itself as "navX-Sensor[0]" in the sim device layer
            self._sim_device = SimDeviceSim("navX-Sensor[0]")
            self._sim_yaw = self._sim_device.getDouble("Yaw")
            self._sim_angle_reg = self._sim_device.getDouble("Angle")
            self._sim_rate = self._sim_device.getDouble("Rate")
            self._sim_pitch = self._sim_device.getDouble("Pitch")
            self._sim_roll = self._sim_device.getDouble("Roll")
            self._sim_connected = self._sim_device.getBoolean("IsConnected")
            self._sim_connected.set(True)

    def execute(self) -> None:
        pass

    # ── Simulation ─────────────────────────────────────────────────────────────

    def sim_set_angle(self, angle: float) -> None:
        """Set cumulative angle in degrees (CCW positive, your convention)."""
        if RobotBase.isSimulation():
            self._sim_angle = angle
            self._sim_angle_reg.set(-angle)  # un-negate back to navX convention
            self._sim_yaw.set(-angle % 360)

    def sim_set_rate(self, rate: float) -> None:
        """Set turn rate in degrees/sec (CCW positive, your convention)."""
        if RobotBase.isSimulation():
            self._sim_rate.set(-rate)

    def sim_set_pitch(self, pitch: float) -> None:
        if RobotBase.isSimulation():
            self._sim_pitch.set(pitch)

    def sim_set_roll(self, roll: float) -> None:
        if RobotBase.isSimulation():
            self._sim_roll.set(roll)

    # ── Heading ────────────────────────────────────────────────────────────────

    def get_angle(self) -> float:
        return -self._navx.getAngle()

    def get_rotation2d(self) -> Rotation2d:
        return Rotation2d.fromDegrees(self.get_angle())

    def get_yaw(self) -> float:
        return -self._navx.getYaw()

    # ── Rates ──────────────────────────────────────────────────────────────────

    @feedback
    def get_turn_rate(self) -> float:
        return -self._navx.getRate()

    # ── Tilt ───────────────────────────────────────────────────────────────────

    @feedback
    def get_pitch(self) -> float:
        return self._navx.getPitch()

    @feedback
    def get_roll(self) -> float:
        return self._navx.getRoll()

    # ── Acceleration ───────────────────────────────────────────────────────────

    def get_world_accel_x(self) -> float:
        return self._navx.getWorldLinearAccelX()

    def get_world_accel_y(self) -> float:
        return self._navx.getWorldLinearAccelY()

    # ── Status ─────────────────────────────────────────────────────────────────

    @feedback
    def is_connected(self) -> bool:
        return self._navx.isConnected()

    @feedback
    def is_calibrating(self) -> bool:
        return self._navx.isCalibrating()

    # ── Control ────────────────────────────────────────────────────────────────

    def reset(self) -> None:
        self._navx.reset()
        if RobotBase.isSimulation():
            self.sim_set_angle(0.0)

    def zero_yaw(self) -> None:
        self._navx.zeroYaw()
