import commands2
import constants
import wpilib


class Drive(commands2.SubsystemBase):
    def __init__(self):
        super().__init__()
        # TODO: Set up real motor cotrollers - change PWMSparkMax with rev.CANSparkMax or ctre.WPI_TalonSRX
        # AM14U4 Kitbot with 2 motors per side
        self.left_front = wpilib.PWMSparkMax(constants.LEFT_FRONT_MOTOR)
        self.left_back = wpilib.PWMSparkMax(constants.LEFT_BACK_MOTOR)
        self.right_front = wpilib.PWMSparkMax(constants.RIGHT_FRONT_MOTOR)
        self.right_back = wpilib.PWMSparkMax(constants.RIGHT_BACK_MOTOR)

        # Group motors
        self.left_group = wpilib.MotorControllerGroup(self.left_front, self.left_back)
        self.right_group = wpilib.MotorControllerGroup(self.right_front, self.right_back)

        # Reverse one side if needed
        self.right_group.setInverted(True)
        self.drive = wpilib.drive.DifferentialDrive(self.left_group, self.right_group)

    def arcade_drive(self, fwd, rot):
        self.drive.arcadeDrive(fwd, rot)
