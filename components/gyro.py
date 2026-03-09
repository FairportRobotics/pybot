from navx import AHRS
from phoenix6.hardware import Pigeon2
from wpimath.geometry import Rotation2d
import wpilib


class NavX2:
    SPI_PORT: wpilib.SPI.Port.kMXP
    USE_FUSED_HEADING: bool = False

    def setup(self) -> None:
        self.navx = AHRS(self.SPI_PORT)
        self.reset()

    def heading(self) -> float:
        if self.USE_FUSED_HEADING:
            return self.navx.getFusedHeading()
        # return self.navx.getYaw()
        return self.navx.getCompassHeading()

    def reset(self) -> None:
        self.navx.reset()

    def execute(self) -> None:
        pass


class Pigeon:
    # MagicBot will inject this from the Robot class
    DEVICE_ID: int
    CANBUS: str = ""

    def setup(self):
        """Called after injection but before the first loop."""
        # Optional: reset yaw to 0 on startup
        self.pigeon2 = Pigeon2(self.DEVICE_ID, self.CANBUS)
        self._yaw = 0.0
        self.reset()

    def execute(self):
        """Standard MagicBot loop. Updates internal state."""
        self._yaw = self.pigeon2.get_yaw().value

    def yaw(self) -> float:
        """Returns the current yaw in degrees."""
        return self._yaw

    def rotation2d(self) -> Rotation2d:
        """Returns a Rotation2d object for WPILib odometry."""
        return Rotation2d.fromDegrees(self._yaw)

    def reset(self):
        """Resets the yaw to 0."""
        self._yaw = 0.0
        self.pigeon2.set_yaw(0)
