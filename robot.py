import components
import constants
from magicbot import MagicRobot
from phoenix6.hardware import TalonFX, CANcoder
from phoenix6.configs import (
    TalonFXConfiguration,
    CANcoderConfiguration,
)
from phoenix6.signals import (
    NeutralModeValue,
    FeedbackSensorSourceValue,
    AbsoluteSensorRangeValue,
    SensorDirectionValue,
)


class MyRobot(MagicRobot):
    scribe: components.Scribe
    accelerometer: components.RoboRioAccelerometer
    gyro: components.NavX2
    led: components.LED
    limelight: components.Limelight
    main_controller: components.XboxController
    '''
    front_left_swerve_module: components.SwerveModule
    front_right_swerve_module: components.SwerveModule
    rear_left_swerve_module: components.SwerveModule
    rear_right_swerve_module: components.SwerveModule
    drivetrain: components.SwerveDrive
    #'''

    def createObjects(self):
        """Create motors and stuff here"""
        # Controller stuff here
        self.main_controller_correct_for_deadband = (
            constants.CONTROLLER_CORRECT_FOR_DEADBAND
        )
        self.main_controller_deadband = constants.CONTROLLER_DEADBAND
        self.main_controller_port = constants.CONTROLLER_PORT

        # LED stuff here
        self.led_length = constants.LED_LENGTH
        self.led_pwm_port = constants.LED_PWM_PORT
    
        # Swerve Module stuff here
        ## front left
        self.front_left_swerve_module_drive_motor = self._create_drive_motor(
            constants.CAN_IDS["front_left_drive"]
        )
        self.front_left_swerve_module_steer_motor = self._create_steer_motor(
            constants.CAN_IDS["front_left_steer"],
            constants.CAN_IDS["front_left_cancoder"],
        )
        self.front_left_swerve_module_cancoder = self._create_can_coder(
            constants.CAN_IDS["front_left_cancoder"],
            constants.CANCODER_OFFSETS["front_left"],
        )
        ## front right
        self.front_right_swerve_module_drive_motor = self._create_drive_motor(
            constants.CAN_IDS["front_right_drive"]
        )
        self.front_right_swerve_module_steer_motor = self._create_steer_motor(
            constants.CAN_IDS["front_right_steer"],
            constants.CAN_IDS["front_right_cancoder"],
        )
        self.front_right_swerve_module_cancoder = self._create_can_coder(
            constants.CAN_IDS["front_right_cancoder"],
            constants.CANCODER_OFFSETS["front_right"],
        )
        ## rear left
        self.rear_left_swerve_module_drive_motor = self._create_drive_motor(
            constants.CAN_IDS["rear_left_drive"]
        )
        self.rear_left_swerve_module_steer_motor = self._create_steer_motor(
            constants.CAN_IDS["rear_left_steer"],
            constants.CAN_IDS["rear_left_cancoder"],
        )
        self.rear_left_swerve_module_cancoder = self._create_can_coder(
            constants.CAN_IDS["rear_left_cancoder"],
            constants.CANCODER_OFFSETS["rear_left"],
        )
        ## rear right
        self.rear_right_swerve_module_drive_motor = self._create_drive_motor(
            constants.CAN_IDS["rear_right_drive"]
        )
        self.rear_right_swerve_module_steer_motor = self._create_steer_motor(
            constants.CAN_IDS["rear_right_steer"],
            constants.CAN_IDS["rear_right_cancoder"],
        )
        self.rear_right_swerve_module_cancoder = self._create_can_coder(
            constants.CAN_IDS["rear_right_cancoder"],
            constants.CANCODER_OFFSETS["rear_right"],
        )
        # Gyro stuff here
        self.drivetrain_gyro = self.gyro

    def teleopInit(self):
        """Called when teleop starts; optional"""
        pass

    def teleopPeriodic(self):
        self.main_controller.capture_button_presses()
        left_x, left_y, right_x, right_y = self.main_controller.get_joysticks()

    def disabledInit(self):
        pass

    def disabledPeriodic(self):
        self.led.turn_off()

    # Helper methods for creating/configuring hardware; these are called from createObjects() to keep that method cleaner
    def _create_can_coder(self, cancoder_id: int, offset: float) -> CANcoder:
        cancoder = CANcoder(cancoder_id)
        cc_cfg = CANcoderConfiguration()
        cc_cfg.magnet_sensor.absolute_sensor_range = (
            AbsoluteSensorRangeValue.SIGNED_PLUS_MINUS_HALF
        )
        cc_cfg.magnet_sensor.sensor_direction = (
            SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE
        )
        cc_cfg.magnet_sensor.magnet_offset = offset
        cancoder.configurator.apply(cc_cfg)
        return cancoder

    def _create_drive_motor(self, drive_id: int) -> TalonFX:
        drive_motor = TalonFX(drive_id)
        drive_cfg = TalonFXConfiguration()
        drive_cfg.motor_output.neutral_mode = NeutralModeValue.BRAKE
        drive_cfg.slot0.k_p = constants.PID["drive"]["P"]
        drive_cfg.slot0.k_i = constants.PID["drive"]["I"]
        drive_cfg.slot0.k_d = constants.PID["drive"]["D"]
        drive_cfg.slot0.k_v = constants.DRIVE_V
        drive_cfg.feedback.sensor_to_mechanism_ratio = constants.DRIVE_GEAR_RATIO
        drive_motor.configurator.apply(drive_cfg)
        return drive_motor

    def _create_steer_motor(self, steer_id: int, cancoder_id: int) -> TalonFX:
        steer_motor = TalonFX(steer_id)
        steer_cfg = TalonFXConfiguration()
        steer_cfg.motor_output.neutral_mode = NeutralModeValue.BRAKE
        steer_cfg.slot0.k_p = constants.PID["steer"]["P"]
        steer_cfg.slot0.k_i = constants.PID["steer"]["I"]
        steer_cfg.slot0.k_d = constants.PID["steer"]["D"]
        # Fuse the CANcoder so the TalonFX uses absolute position directly
        steer_cfg.feedback.feedback_remote_sensor_id = cancoder_id
        steer_cfg.feedback.feedback_sensor_source = (
            FeedbackSensorSourceValue.FUSED_CA_NCODER
        )
        steer_cfg.feedback.sensor_to_mechanism_ratio = 1.0
        steer_cfg.feedback.rotor_to_sensor_ratio = constants.STEER_GEAR_RATIO
        # Continuous wrap: allow shortest-path steering across ±180°
        steer_cfg.closed_loop_general.continuous_wrap = True
        steer_motor.configurator.apply(steer_cfg)
        return steer_motor
