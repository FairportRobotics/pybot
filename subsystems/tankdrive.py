import commands2
import constants


class TankDrive(commands2.SubsystemBase):
    def __init__(
        self, motor_type: str = "TalonSRX", right_inverted: bool = True
    ) -> None:
        """TankDrive subsystem for controlling the robot's drive motors."""
        super().__init__()

        self.right_inverted = right_inverted

        if motor_type == "TalonSRX":
            import ctre

            self.left_main = ctre.WPI_TalonSRX(constants.CAN_LEFT_DRIVE_MOTOR_1)
            self.left_follower = ctre.WPI_TalonSRX(constants.CAN_LEFT_DRIVE_MOTOR_2)
            self.right_main = ctre.WPI_TalonSRX(constants.CAN_RIGHT_DRIVE_MOTOR_1)
            self.right_follower = ctre.WPI_TalonSRX(constants.CAN_RIGHT_DRIVE_MOTOR_1)
        elif motor_type == "SparkMax":
            import rev

            self.left_main = rev.CANSparkMax(
                constants.CAN_LEFT_DRIVE_MOTOR_1, rev.MotorType.kBrushless
            )
            self.left_follower = rev.CANSparkMax(
                constants.CAN_LEFT_DRIVE_MOTOR_2, rev.MotorType.kBrushless
            )
            self.right_main = rev.CANSparkMax(
                constants.CAN_RIGHT_DRIVE_MOTOR_1, rev.MotorType.kBrushless
            )
            self.right_follower = rev.CANSparkMax(
                constants.CAN_RIGHT_DRIVE_MOTOR_2, rev.MotorType.kBrushless
            )
        else:
            raise ValueError(f"Unsupported motor type: {motor_type}")

    def periodic(self):
        pass  # Called every robot loop

    def drive(self, left: float, right: float):
        if self.right_inverted:
            right = -right
        else:
            left = -left

        self.left_main.set(left)
        self.left_follower.set(left)

        self.right_main.set(right)
        self.right_follower.set(right)
