from wpimath.geometry import Rotation2d

class PseudoGyro:
    def heading(self):
        return Rotation2d.fromDegrees(0.0)

    def reset(self):
        pass

    def get_yaw(self):
        class Yaw:
            value = 0.0
        return Yaw()

    def execute(self):
        pass