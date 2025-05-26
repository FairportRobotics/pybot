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

            self.left_front = ctre.WPI_TalonSRX(
                constants.LEFT_FRONT_DRIVE_MOTOR_CAN_BUS_ID
            )
            self.left_rear = ctre.WPI_TalonSRX(
                constants.LEFT_REAR_DRIVE_MOTOR_CAN_BUS_ID
            )
            self.right_front = ctre.WPI_TalonSRX(
                constants.RIGHT_FRONT_DRIVE_MOTOR_CAN_BUS_ID
            )
            self.right_rear = ctre.WPI_TalonSRX(
                constants.RIGHT_REAR_DRIVE_MOTOR_CAN_BUS_ID
            )
        elif motor_type == "SparkMax":
            import rev

            self.left_front = rev.CANSparkMax(
                constants.LEFT_FRONT_DRIVE_MOTOR_CAN_BUS_ID, rev.MotorType.kBrushless
            )
            self.left_rear = rev.CANSparkMax(
                constants.LEFT_REAR_DRIVE_MOTOR_CAN_BUS_ID, rev.MotorType.kBrushless
            )
            self.right_front = rev.CANSparkMax(
                constants.RIGHT_FRONT_DRIVE_MOTOR_CAN_BUS_ID, rev.MotorType.kBrushless
            )
            self.right_rear = rev.CANSparkMax(
                constants.RIGHT_REAR_DRIVE_MOTOR_CAN_BUS_ID, rev.MotorType.kBrushless
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

        self.left_front.set(left)
        self.left_rear.set(left)

        self.right_front.set(right)
        self.right_rear.set(right)
