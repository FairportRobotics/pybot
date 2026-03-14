import wpilib
from magicbot import feedback
import time


class RoboRioAccelerometer:
    SWITCH_X_AND_Y: bool = False

    def setup(self) -> None:
        self.accelerometer = wpilib.BuiltInAccelerometer()
        self.x_accel = 0.0
        self.y_accel = 0.0
        self.z_accel = 0.0
        self.last_time = time.monotonic()
        self.G_TO_MPS2 = 9.81
        self.velocity = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.distance = {"x": 0.0, "y": 0.0, "z": 0.0}

    def execute(self):
        now = time.monotonic()
        dt = now - self.last_time
        self.last_time = now

        # Get the acceleration values
        if self.SWITCH_X_AND_Y:
            self.x_accel = self.accelerometer.getY()
            self.y_accel = self.accelerometer.getX()
        else:
            self.x_accel = self.accelerometer.getX()
            self.y_accel = self.accelerometer.getY()
        self.z_accel = self.accelerometer.getZ()

        # Convert from G to M/Second squared
        acceleration = {
            "x": self.x_accel * self.G_TO_MPS2,
            "y": self.y_accel * self.G_TO_MPS2,
            "z": self.z_accel * self.G_TO_MPS2,
        }

        for key, value in acceleration.items():
            # Simple deadband to reduce noise
            if abs(value) < 0.1:
                acceleration[key] = 0
            # Double Integration
            # velocity = velocity + (acceleration * dt)
            self.velocity[key] += acceleration[key] * dt
            # distance = distance + (velocity * dt)
            self.distance[key] += self.velocity[key] * dt

    # =========================================================================
    # CONTROL METHODS
    # =========================================================================

    def reset(self) -> None:
        self.last_time = time.monotonic()
        self.velocity = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.distance = {"x": 0.0, "y": 0.0, "z": 0.0}

    # =========================================================================
    # INFORMATIONAL METHODS
    # =========================================================================

    @feedback
    def distance_x(self):
        return self.distance["x"]

    @feedback
    def distance_y(self):
        return self.distance["y"]

    @feedback
    def distance_z(self):
        return self.distance["z"]

    @feedback
    def raw_x(self):
        return self.x_accel

    @feedback
    def raw_y(self):
        return self.y_accel

    @feedback
    def raw_z(self):
        return self.z_accel

    @feedback
    def velocity_x(self):
        return self.velocity["x"]

    @feedback
    def velocity_y(self):
        return self.velocity["y"]

    @feedback
    def velocity_z(self):
        return self.velocity["z"]
