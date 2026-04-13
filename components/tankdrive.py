import constants
from wpilib.drive import DifferentialDrive
import rev


class TankDrive:
    def setup(self):
        self.leftLeader = rev.SparkMax(
            constants.CAN_IDs.LEFT_MOTOR, rev.SparkLowLevel.MotorType.kBrushed
        )
        self.leftFollower = rev.SparkMax(
            constants.CAN_IDs.LEFT_FOLLOWER_MOTOR, rev.SparkLowLevel.MotorType.kBrushed
        )
        self.rightLeader = rev.SparkMax(
            constants.CAN_IDs.RIGHT_MOTOR, rev.SparkLowLevel.MotorType.kBrushed
        )
        self.rightFollower = rev.SparkMax(
            constants.CAN_IDs.RIGHT_FOLLOWER_MOTOR, rev.SparkLowLevel.MotorType.kBrushed
        )

        # Set can timeout. Because this project only sets parameters once on
        # construction, the timeout can be long without blocking robot operation.
        self.leftLeader.setCANTimeout(250)
        self.rightLeader.setCANTimeout(250)
        self.leftFollower.setCANTimeout(250)
        self.rightFollower.setCANTimeout(250)

        # Create the configuration to apply to motors. Voltage compensation helps
        # the robot perform more similarly on different battery voltages.
        config = rev.SparkMaxConfig()
        config.voltageCompensation(12)
        config.smartCurrentLimit(constants.TankDrive.DRIVE_MOTOR_CURRENT_LIMIT)

        # Set configuration to follow each leader and then apply it to corresponding
        # follower.
        config.follow(self.leftLeader)
        self.leftFollower.configure(
            config,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )
        config.follow(self.rightLeader)
        self.rightFollower.configure(
            config,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )

        # Remove following, then apply config to right leader
        config.disableFollowerMode()
        self.rightLeader.configure(
            config,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )
        # Set config to inverted and then apply to left leader. Set Left side
        # inverted so that positive values drive both sides forward
        config.inverted(True)
        self.leftLeader.configure(
            config,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )

        # set up differential drive class
        self.drive = DifferentialDrive(self.leftLeader, self.rightLeader)

    def execute(self):
        pass
