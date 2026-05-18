import components
import constants
from magicbot import MagicRobot, feedback
from phoenix5 import WPI_TalonSRX


class MyRobot(MagicRobot):
    controller: components.XboxController
    drivetrain: components.DriveTrain
    roller: components.Roller

    def createObjects(self):
        self.name = constants.Robot.NAME
        self.controller_port = constants.CONTROLLER_PORT

        # create motors for the drivetrain
        self.drivetrain_left_leader = WPI_TalonSRX(
            constants.CanBusIds.LEFT_LEADER_MOTOR
        )
        self.drivetrain_left_follower = WPI_TalonSRX(
            constants.CanBusIds.LEFT_FOLLOWER_MOTOR
        )
        self.drivetrain_right_leader = WPI_TalonSRX(
            constants.CanBusIds.RIGHT_LEADER_MOTOR
        )
        self.drivetrain_right_follower = WPI_TalonSRX(
            constants.CanBusIds.RIGHT_FOLLOWER_MOTOR
        )
        self.roller_motor = WPI_TalonSRX(constants.CanBusIds.ROLLER_MOTOR)

    def teleopPeriodic(self):
        # =============================================================
        # JOYSTICK HANDLING
        # =============================================================
        if self.controller.b_button_pressed():
            self.roller.speed = self.controller.left_y
        else:
            self.roller.speed = 0
            # Control the drivetrain based on the controller input
            self.drivetrain.speed = self.controller.left_y
            self.drivetrain.rotation = self.controller.right_x

    @feedback(key="name")
    def get_name(self) -> str:
        return self.name
