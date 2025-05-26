# Python port of https://www.chiefdelphi.com/t/the-simplest-frc-setpoint-elevator/492673
import commands2
import rev
import wpilib


class Elevator(commands2.SubsystemBase):
    def __init__(self):
        super().__init__()

        # Replace 0 with your actual motor controller CAN ID
        self.motor = rev.CANSparkMax(0, rev.CANSparkMaxLowLevel.MotorType.kBrushless)
        self.encoder = self.motor.getEncoder()

        # PID gains - these need tuning for your robot
        self.pid = wpilib.PIDController(0.1, 0, 0)

        # Constants (tune for your robot)
        self.ENCODER_COUNTS_PER_INCH = 42.0
        self.GRAVITY_COMPENSATION = 0.1

    def get_height(self):
        """Returns elevator height in inches."""
        return self.encoder.getPosition() / self.ENCODER_COUNTS_PER_INCH

    def set_position(self, target_height):
        """Set elevator to a specific height in inches."""
        pid_output = self.pid.calculate(self.get_height(), target_height)

        # Add gravity compensation (sign may need flipping)
        motor_output = pid_output + self.GRAVITY_COMPENSATION

        # Clamp output between -1.0 and 1.0
        motor_output = max(min(motor_output, 1.0), -1.0)

        self.motor.set(motor_output)

    def periodic(self):
        """Called once per scheduler run."""
        pass
