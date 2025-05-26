import commands2
import constants
import json
import wpilib


class Controller(commands2.Subsystem):
    def __init__(self, controller_type: str = "xbox") -> None:
        """
        Initialize the controller subsystem.
        """
        super().__init__()
        self.controller_type = controller_type.lower()
        if self.controller_type == "xbox":
            self.device = wpilib.XboxController(constants.CONTROLLER_PORT)
            # Initialize the buttons for the Xbox controller
            self.a_button = commands2.button.JoystickButton(
                self.device, wpilib.XboxController.Button.kA
            )
            self.b_button = commands2.button.JoystickButton(
                self.device, wpilib.XboxController.Button.kB
            )
            self.x_button = commands2.button.JoystickButton(
                self.device, wpilib.XboxController.Button.kX
            )
            self.y_button = commands2.button.JoystickButton(
                self.device, wpilib.XboxController.Button.kY
            )
            # The bumpers
            self.left_bumper = commands2.button.JoystickButton(
                self.device, wpilib.XboxController.Button.kLeftBumper
            )
            self.right_bumper = commands2.button.JoystickButton(
                self.device, wpilib.XboxController.Button.kRightBumper
            )
            # The sticks
            self.left_stick = commands2.button.JoystickButton(
                self.device, wpilib.XboxController.Button.kLeftStick
            )
            self.right_stick = commands2.button.JoystickButton(
                self.device, wpilib.XboxController.Button.kRightStick
            )
            # The D-pad
            self.dpad_up = commands2.button.POVButton(self.device, 0)
            self.dpad_right = commands2.button.POVButton(self.device, 90)
            self.dpad_down = commands2.button.POVButton(self.device, 180)
            self.dpad_left = commands2.button.POVButton(self.device, 270)

            self.start_button = commands2.button.JoystickButton(
                self.device, wpilib.XboxController.Button.kStart
            )

            self.back_button = commands2.button.JoystickButton(
                self.device, wpilib.XboxController.Button.kBack
            )
        elif self.controller_type == "ps4":
            self.device = wpilib.PS4Controller(constants.CONTROLLER_PORT)
            # Initialize the buttons for the PS4 controller
            self.cross_button = commands2.button.JoystickButton(
                self.device, wpilib.PS4Controller.Button.kCross
            )
            self.circle_button = commands2.button.JoystickButton(
                self.device, wpilib.PS4Controller.Button.kCircle
            )
            self.square_button = commands2.button.JoystickButton(
                self.device, wpilib.PS4Controller.Button.kSquare
            )
            self.triangle_button = commands2.button.JoystickButton(
                self.device, wpilib.PS4Controller.Button.kTriangle
            )
            self.left_bumper = commands2.button.JoystickButton(
                self.device, wpilib.PS4Controller.Button.kL1
            )
            self.right_bumper = commands2.button.JoystickButton(
                self.device, wpilib.PS4Controller.Button.kR1
            )
            self.left_stick = commands2.button.JoystickButton(
                self.device, wpilib.PS4Controller.Button.kL3
            )
            self.right_stick = commands2.button.JoystickButton(
                self.device, wpilib.PS4Controller.Button.kR3
            )
            self.share_button = commands2.button.JoystickButton(
                self.device, wpilib.PS4Controller.Button.kShare
            )
            self.options_button = commands2.button.JoystickButton(
                self.device, wpilib.PS4Controller.Button.kOptions
            )
        else:
            raise ValueError(f"Unsupported controller type: {self.controller_type}")

        # Load drift compensation values from a JSON file if it exists
        try:
            with open("joystick_drift.json", "r") as f:
                self.drift = json.load(f)
            self.drift_calibrated = True
        except FileNotFoundError:
            self.drift = {
                "left_x": 0.0,
                "left_y": 0.0,
                "right_x": 0.0,
                "right_y": 0.0,
            }
            self.drift_calibrated = False

    def get_left_stick_x(self) -> float:
        """
        Get the x-axis value of the left stick.
        """
        return self.get_stick(constants.LEFT_JOYSTICK_X, "left_x")

    def get_left_stick_y(self) -> float:
        """
        Get the y-axis value of the left stick.
        """
        return self.get_stick(constants.LEFT_JOYSTICK_Y, "left_y")

    def get_right_stick_x(self) -> float:
        """
        Get the x-axis value of the right stick.
        """
        return self.get_stick(constants.RIGHT_JOYSTICK_X, "right_x")

    def get_right_stick_y(self) -> float:
        """
        Get the y-axis value of the right stick.
        """
        return self.get_stick(constants.RIGHT_JOYSTICK_Y, "right_y")

    def get_stick(self, axis: int, drift_key: str) -> float:
        """
        Generalized function to read the X or y-axis of either the left or right stick and apply drift compensation.

        :param axis: The axis to read (e.g., LEFT_JOYSTICK_X, LEFT_JOYSTICK_Y, etc.)
        :param drift_key: The key for the drift compensation value.
        :return: The compensated stick value.
        """
        stick = self.device.getRawAxis(axis)
        drift = self.drift.get(drift_key, 0.0)
        return stick - drift
