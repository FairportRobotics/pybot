import constants
from wpilib.drive import DifferentialDrive
import rev
from magicbot import will_reset_to


class TankDrive:
    _speed = will_reset_to(0.0)
    _rotation = will_reset_to(0.0)

    def setup(self):
        self.left_leader = rev.SparkMax(
            constants.CAN_IDs.LEFT_MOTOR, rev.SparkLowLevel.MotorType.kBrushed
        )
        self.left_follower = rev.SparkMax(
            constants.CAN_IDs.LEFT_FOLLOWER_MOTOR, rev.SparkLowLevel.MotorType.kBrushed
        )
        self.right_leader = rev.SparkMax(
            constants.CAN_IDs.RIGHT_MOTOR, rev.SparkLowLevel.MotorType.kBrushed
        )
        self.right_follower = rev.SparkMax(
            constants.CAN_IDs.RIGHT_FOLLOWER_MOTOR, rev.SparkLowLevel.MotorType.kBrushed
        )

        # Set can timeout. Because this project only sets parameters once on
        # construction, the timeout can be long without blocking robot operation.
        self.left_leader.setCANTimeout(constants.TankDrive.CAN_TIMEOUT_MS)
        self.right_leader.setCANTimeout(constants.TankDrive.CAN_TIMEOUT_MS)
        self.left_follower.setCANTimeout(constants.TankDrive.CAN_TIMEOUT_MS)
        self.right_follower.setCANTimeout(constants.TankDrive.CAN_TIMEOUT_MS)

        # Create the configuration to apply to motors. Voltage compensation helps
        # the robot perform more similarly on different battery voltages.
        config = rev.SparkMaxConfig()
        config.voltageCompensation(constants.TankDrive.VOLTAGE_COMPENSATION)
        config.smartCurrentLimit(constants.TankDrive.DRIVE_MOTOR_CURRENT_LIMIT)

        # Set configuration to follow each leader and then apply it to corresponding
        # follower.
        config.follow(self.left_leader)
        self.left_follower.configure(
            config,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )
        config.follow(self.right_leader)
        self.right_follower.configure(
            config,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )

        # Remove following, then apply config to right leader
        config.disableFollowerMode()
        self.right_leader.configure(
            config,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )
        # Set config to inverted and then apply to left leader. Set Left side
        # inverted so that positive values drive both sides forward
        config.inverted(True)
        self.left_leader.configure(
            config,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )

        # set up differential drive class
        self._drive = DifferentialDrive(self.left_leader, self.right_leader)

    def execute(self):
        self._drive.arcadeDrive(self._speed, self._rotation)

    def drive(self, speed: float, rotation: float) -> None:
        self._speed = speed
        self._rotation = rotation
