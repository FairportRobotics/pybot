from magicbot import feedback
import navx
from wpimath.geometry import Rotation2d, Rotation3d, Pose2d


class NavX2:
    USE_FUSED_HEADING: bool = True

    def execute(self) -> None:
        """Standard MagicBot loop. Updates internal state."""
        self._pitch = self.gyro.getPitch()
        self._roll = self.gyro.getRoll()
        self._yaw = self.heading()
        self._rotation2d = self.gyro.getRotation2d()
        self._rotation3d = self.gyro.getRotation3d()
        self._last_update = self.gyro.getLastSensorTimestamp()

    def setup(self) -> None:
        """Called after injection but before the first loop."""
        self.gyro = navx.AHRS.create_spi()
        self.reset()
        self._pitch = 0.0
        self._roll = 0.0
        self._yaw = 0.0
        self._last_update = 0.0
        self._rotation2d = self.gyro.getRotation2d()
        self._rotation3d = self.gyro.getRotation3d()

    # =========================================================================
    # CONTROL METHODS
    # =========================================================================

    def reset(self) -> None:
        self._yaw = 0.0
        self.gyro.reset()
        self.gyro.zeroYaw()

    # =========================================================================
    # INFORMATIONAL METHODS
    # =========================================================================

    @feedback
    def calibrated(self) -> bool:
        return self.gyro.isCalibrating() == False

    @feedback
    def connected(self) -> bool:
        return self.gyro.isConnected()

    def heading(self) -> float:
        if self.USE_FUSED_HEADING:
            return self.gyro.getFusedHeading()
        # return self.gyro.getYaw()
        return self.gyro.getCompassHeading()

    @feedback
    def magnetic_disturbance(self) -> bool:
        return self.gyro.isMagneticDisturbance()

    @feedback
    def magnetometer_calibrated(self) -> bool:
        return self.gyro.isMagnetometerCalibrated()

    @feedback
    def last_updated(self) -> float:
        return self._last_update

    @feedback
    def pitch(self) -> float:
        return self._pitch

    @feedback
    def roll(self) -> float:
        return self._roll

    def rotation2d(self) -> Rotation2d:
        """Returns a Rotation2d object for WPILib odometry."""
        return self._rotation2d

    def rotation3d(self) -> Rotation3d:
        """Returns a 3D rotation object for WPILib odometry."""
        return self._rotation3d

    @feedback
    def yaw(self) -> float:
        return self._yaw
