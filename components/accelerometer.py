import wpilib
from magicbot import feedback
import math
import time


class RoboRioAccelerometer:
    switch_x_and_y: bool = False

    def setup(self) -> None:
        self.accelerometer = wpilib.BuiltInAccelerometer()
        self.x_accel = 0.0
        self.y_accel = 0.0
        self.z_accel = 0.0
        self.last_time = time.monotonic()
        self.G_TO_MPS2 = 9.80665  # 9.81
        self.velocity = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.distance = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.pitch = 0.0
        self.roll = 0.0
        self.yaw = 0.0
        self.quaternion = {"w": 0.0, "x": 0.0, "y": 0.0, "z": 0.0}

    def execute(self):
        now = time.monotonic()
        dt = now - self.last_time
        self.last_time = now

        # Get the acceleration values
        if self.switch_x_and_y:
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

        norm = math.sqrt(
            acceleration["x"] ** 2 + acceleration["y"] ** 2 + acceleration["z"] ** 2
        )
        a_prime = {
            "x": acceleration["x"] / norm,
            "y": acceleration["y"] / norm,
            "z": acceleration["z"] / norm,
        }
        self.pitch = math.atan2(
            a_prime["x"], math.sqrt(a_prime["y"] ** 2 + a_prime["z"] ** 2)
        )
        self.roll = math.atan2(-a_prime["y"], -a_prime["z"])
        self.quaternion["w"] = math.cos(self.roll / 2) * math.cos(self.pitch / 2)
        self.quaternion["x"] = math.sin(self.roll / 2) * math.cos(self.pitch / 2)
        self.quaternion["y"] = math.cos(self.roll / 2) * math.sin(self.pitch / 2)
        self.quaternion["z"] = -math.sin(self.roll / 2) * math.sin(self.pitch / 2)

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

    @feedback(key="pitch")
    def the_pitch(self):
        return self.pitch

    @feedback(key="roll")
    def the_roll(self):
        return self.roll

    @feedback
    def quaternion_w(self):
        return self.quaternion["w"]

    @feedback
    def quaternion_x(self):
        return self.quaternion["x"]

    @feedback
    def quaternion_y(self):
        return self.quaternion["y"]

    @feedback
    def quaternion_z(self):
        return self.quaternion["z"]
