from navx import AHRS
from commands2 import SubsystemBase
from wpilib import SmartDashboard
import time


class GyroSubsystem(SubsystemBase):
    def __init__(self, name: str = "GyroSubsystem"):
        super().__init__()
        self.name = name
        self.navx = None

    def initialize(self):
        try:
            # Initialize the NavX sensor on SPI (similar to kMXP_SPI in Java)
            self.navx = AHRS.create_spi()
            self.navx.enable_boardlevel_yaw_reset(True)

            # Wait until calibration completes
            while self.navx.is_calibrating():
                time.sleep(0.01)  # small delay to avoid busy waiting

            self.set_to_zero()
            print("GyroSubsystem initialized successfully.")
        except RuntimeError as ex:
            print(f"Failed to initialize NavX: {ex}")

    def get_heading(self) -> float:
        angle = self.navx.get_fused_heading()
        print(angle)
        return angle

    def set_to_zero(self):
        self.navx.zero_yaw()

    def get_angle_pid_get(self) -> float:
        return self.navx.get_angle()

    def get_rate(self) -> float:
        # Return the rate of rotation of the yaw (Z-axis) gyro, in degrees per second.
        return self.navx.get_yaw_rate()

    def get_tilt(self) -> float:
        return self.navx.get_pitch()

    def is_connected(self) -> bool:
        return self.navx.is_connected()

    def reset(self):
        self.navx.reset()

    def update_dashboard(self):
        SmartDashboard.putNumber("gyro.heading", self.get_heading())
