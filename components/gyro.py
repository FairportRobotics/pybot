from magicbot import feedback
from navx import AHRS
from phoenix6.hardware import Pigeon2
from wpimath.geometry import Rotation2d, Rotation3d
import wpilib


class NavX2:
    SPI_PORT: wpilib.SPI.Port.kMXP
    USE_FUSED_HEADING: bool = True

    def execute(self) -> None:
        """Standard MagicBot loop. Updates internal state."""
        self._pitch = self.navx.getPitch()
        self._roll = self.navx.getRoll()
        self._yaw = self.heading()
        self._rotation2d = self.navx.getRotation2d()
        self._rotation3d = self.navx.getRotation3d()

    def setup(self) -> None:
        """Called after injection but before the first loop."""
        self.navx = AHRS(self.SPI_PORT)
        self.reset()
        self._pitch = 0.0
        self._roll = 0.0
        self._yaw = 0.0
        self._rotation2d = self.navx.getRotation2d()
        self._rotation3d = self.navx.getRotation3d()

    # =========================================================================
    # CONTROL METHODS
    # =========================================================================

    def reset(self) -> None:
        self._yaw = 0.0
        self.navx.reset()
        self.navx.zeroYaw()

    # =========================================================================
    # INFORMATIONAL METHODS
    # =========================================================================

    @feedback
    def calibrated(self) -> bool:
        return self.navx.isCalibrating() == False

    @feedback
    def connected(self) -> bool:
        return self.navx.isConnected()

    def heading(self) -> float:
        if self.USE_FUSED_HEADING:
            return self.navx.getFusedHeading()
        # return self.navx.getYaw()
        return self.navx.getCompassHeading()

    @feedback
    def magnetic_disturbance(self) -> bool:
        return self.navx.isMagneticDisturbance()

    @feedback
    def magnetometer_calibrated(self) -> bool:
        return self.navx.isMagnetometerCalibrated()

    def pitch(self) -> float:
        return self._pitch

    def roll(self) -> float:
        return self._roll

    def rotation2d(self) -> Rotation2d:
        """Returns a Rotation2d object for WPILib odometry."""
        return self._rotation2d

    def rotation3d(self) -> Rotation3d:
        """Returns a 3D rotation object for WPILib odometry."""
        return self._rotation3d

    def yaw(self) -> float:
        return self._yaw


class Pigeon:
    # MagicBot will inject this from the Robot class
    DEVICE_ID: int
    CANBUS: str = ""

    def execute(self):
        """Standard MagicBot loop. Updates internal state."""
        self._pitch = self.pigeon2.get_pitch().value
        self._roll = self.pigeon2.get_roll().value
        self._yaw = self.pigeon2.get_yaw().value
        self._rotation2d = Rotation2d.fromDegrees(self._yaw)
        self._rotation3d = Rotation3d.fromDegrees(self._pitch, self._roll, self._yaw)

    def setup(self):
        """Called after injection but before the first loop."""
        # Optional: reset yaw to 0 on startup
        self.pigeon2 = Pigeon2(self.DEVICE_ID, self.CANBUS)
        self._pitch = 0.0
        self._roll = 0.0
        self._yaw = 0.0
        self._rotation2d = Rotation2d.fromDegrees(self._yaw)
        self._rotation3d = Rotation3d.fromDegrees(self._pitch, self._roll, self._yaw)
        self.reset()

    # =========================================================================
    # CONTROL METHODS
    # =========================================================================

    def reset(self):
        """Resets the yaw to 0."""
        self._yaw = 0.0
        self.pigeon2.set_yaw(0)

    # =========================================================================
    # INFORMATIONAL METHODS
    # =========================================================================

    def heading(self) -> float:
        return self._yaw

    def rotation2d(self) -> Rotation2d:
        """Returns a Rotation2d object for WPILib odometry."""
        return self._rotation2d

    def rotation3d(self) -> Rotation3d:
        """Returns a 3D rotation object for WPILib odometry."""
        return self._rotation3d

    def pitch(self) -> float:
        return self._pitch

    def roll(self) -> float:
        return self._roll

    def yaw(self) -> float:
        return self._yaw
