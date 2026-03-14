from magicbot import feedback
import navx
from wpimath.geometry import Rotation2d, Rotation3d


class NavX2:
    USE_FUSED_HEADING: bool = True
    SWITCH_X_AND_Y: bool = False
    ROBOT_CENTRIC: bool = False

    def execute(self) -> None:
        """Standard MagicBot loop. Updates internal state."""
        if self.is_connected:
            if self.SWITCH_X_AND_Y:
                self._pitch = self.gyro.getRoll()
                self._roll = self.gyro.getPitch()
            else:
                self._pitch = self.gyro.getPitch()
                self._roll = self.gyro.getRoll()
            self._yaw = self.heading()
            self._rotation2d = self.gyro.getRotation2d()
            self._rotation3d = self.gyro.getRotation3d()
            self._last_update = self.gyro.getLastSensorTimestamp()
        else:
            try:
                self.is_connected = self.gyro.isConnected()
            except Exception as e:
                pass

    def setup(self) -> None:
        """Called after injection but before the first loop."""
        self._pitch = 0.0
        self._roll = 0.0
        self._yaw = 0.0
        self._last_update = 0.0
        self.is_connected = False
        try:
            self.gyro = navx.AHRS.create_spi()
            self.reset()
            self._rotation2d = self.gyro.getRotation2d()
            self._rotation3d = self.gyro.getRotation3d()
        except Exception as e:
            self._rotation2d = Rotation2d(0, 0)
            self._rotation3d = Rotation3d(0, 0, 0)

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
        if self.is_connected:
            return self.gyro.isCalibrating() == False
        return False

    @feedback
    def connected(self) -> bool:
        return self.is_connected

    @feedback
    def heading(self) -> float:
        if self.is_connected:
            if self.USE_FUSED_HEADING:
                return self.gyro.getFusedHeading()
            return self.gyro.getCompassHeading()
        return 0.0

    def magnetic_disturbance(self) -> bool:
        if self.is_connected:
            return self.gyro.isMagneticDisturbance()
        return False

    def magnetometer_calibrated(self) -> bool:
        if self.is_connected:
            return self.gyro.isMagnetometerCalibrated()
        return False

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

    @feedback
    def distance_x(self):
        if self.SWITCH_X_AND_Y:
            return self.gyro.getDisplacementY()
        return self.gyro.getDisplacementX()

    @feedback
    def distance_y(self):
        if self.SWITCH_X_AND_Y:
            return self.gyro.getDisplacementX()
        return self.gyro.getDisplacementY()

    @feedback
    def distance_z(self):
        return self.gyro.getDisplacementZ()

    @feedback
    def velocity_x(self):
        if self.ROBOT_CENTRIC:
            if self.SWITCH_X_AND_Y:
                return self.gyro.getRobotCentricVelocityY()
            return self.gyro.getRobotCentricVelocityX()
        else:
            if self.SWITCH_X_AND_Y:
                return self.gyro.getVelocityY()
            return self.gyro.getVelocityX()

    @feedback
    def velocity_y(self):
        if self.ROBOT_CENTRIC:
            if self.SWITCH_X_AND_Y:
                return self.gyro.getRobotCentricVelocityX()
            return self.gyro.getRobotCentricVelocityY()
        else:
            if self.SWITCH_X_AND_Y:
                return self.gyro.getVelocityX()
            return self.gyro.getVelocityY()

    @feedback
    def velocity_z(self):
        return self.gyro.getVelocityZ()
