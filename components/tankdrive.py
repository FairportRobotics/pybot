import wpilib
from wpilib.drive import DifferentialDrive
from magicbot import will_reset_to


class TankDrive:
    mode: str = "tank"
    left_lead_motor: wpilib.Spark
    left_follow_motor: wpilib.Spark
    right_lead_motor: wpilib.Spark
    right_follow_motor: wpilib.Spark
    left_power = will_reset_to(0.0)
    right_power = will_reset_to(0.0)
    forward = will_reset_to(0.0)
    turn = will_reset_to(0.0)
    turn_in_place = will_reset_to(False)

    def setup(self):
        self.left_lead_motor.addFollower(self.left_follow_motor)
        self.left_lead_motor.setInverted(True)
        self.left_lead_motor.setSafetyEnabled(True)
        self.left_lead_motor.setExpiration(0.1)
        self.right_lead_motor.addFollower(self.right_follow_motor)
        self.right_lead_motor.setSafetyEnabled(True)
        self.right_lead_motor.setExpiration(0.1)
        self.drive = DifferentialDrive(self.left_lead_motor, self.right_lead_motor)

    def execute(self):
        if self.mode.lower() == "tank":
            self.drive.tankDrive(self.left_power, self.right_power)
        elif self.mode.lower() == "arcade":
            self.drive.arcadeDrive(self.forward, self.turn)
        elif self.mode.lower() == "curvature":
            self.drive.curvatureDrive(self.forward, self.turn, self.turn_in_place)

    def change_mode(self, mode: str):
        self.mode = mode

    def go(self, left_joystick, right_joystick, turn_in_place=False):
        self.turn_in_place = turn_in_place
        if self.mode.lower() == "tank":
            self.left_power = left_joystick
            self.right_power = right_joystick
        elif self.mode.lower() in ("arcade", "curvature"):
            self.forward = left_joystick
            self.turn = right_joystick

    def stop(self):
        self.left_power = 0.0
        self.right_power = 0.0
        self.forward = 0.0
        self.turn = 0.0
