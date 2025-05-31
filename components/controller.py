import magicbot
import wpilib


class XboxController:
    xbox_controller: wpilib.XboxController

    left_joystick_x = magicbot.tunable(0.0)
    left_joystick_y = magicbot.tunable(0.0)
    right_joystick_x = magicbot.tunable(0.0)
    right_joystick_y = magicbot.tunable(0.0)

    def execute(self) -> None:
        pass

    def get_joysticks(self) -> tuple[float, float, float, float]:
        """
        Get the joystick values from the Xbox controller.

        :return: A tuple containing the left joystick x, left joystick y, right joystick x, and right joystick y values.
        """
        self.left_joystick_x = -round(self.xbox_controller.getLeftX(), 1)
        self.left_joystick_y = -round(self.xbox_controller.getLeftY(), 1)
        self.right_joystick_x = -round(self.xbox_controller.getRightX(), 1)
        self.right_joystick_y = -round(self.xbox_controller.getRightY(), 1)
        return (
            self.left_joystick_x,
            self.left_joystick_y,
            self.right_joystick_x,
            self.right_joystick_y,
        )

    def a_button(self) -> bool:
        """
        Check if the A button is pressed.

        :return: True if the A button is pressed, False otherwise.
        """
        return self.xbox_controller.getAButton()

    def b_button(self) -> bool:
        """
        Check if the B button is pressed.
        :return: True if the B button is pressed, False otherwise.
        """
        return self.xbox_controller.getBButton()

    def x_button(self) -> bool:
        """
        Check if the X button is pressed.

        :return: True if the X button is pressed, False otherwise.
        """
        return self.xbox_controller.getXButton()

    def y_button(self) -> bool:
        """
        Check if the Y button is pressed.

        :return: True if the Y button is pressed, False otherwise.
        """
        return self.xbox_controller.getYButton()

    def dpad_up(self) -> bool:
        """
        Check if the D-pad is pressed up.

        :return: True if the D-pad is pressed up, False otherwise.
        """
        return self.xbox_controller.getPOV() == 0

    def dpad_down(self) -> bool:
        """
        Check if the D-pad is pressed down.

        :return: True if the D-pad is pressed down, False otherwise.
        """
        return self.xbox_controller.getPOV() == 180

    def dpad_left(self) -> bool:
        """
        Check if the D-pad is pressed left.

        :return: True if the D-pad is pressed left, False otherwise.
        """
        return self.xbox_controller.getPOV() == 270

    def dpad_right(self) -> bool:
        """
        Check if the D-pad is pressed right.

        :return: True if the D-pad is pressed right, False otherwise.
        """
        return self.xbox_controller.getPOV() == 90

    def left_bumper(self) -> bool:
        """
        Check if the left bumper is pressed.

        :return: True if the left bumper is pressed, False otherwise.
        """
        return self.xbox_controller.getLeftBumper()

    def right_bumper(self) -> bool:
        """
        Check if the right bumper is pressed.

        :return: True if the right bumper is pressed, False otherwise.
        """
        return self.xbox_controller.getRightBumper()

    def left_trigger(self) -> float:
        """
        Get the value of the left trigger.

        :return: The value of the left trigger, ranging from 0.0 to 1.0.
        """
        return self.xbox_controller.getLeftTriggerAxis()

    def right_trigger(self) -> float:
        """
        Get the value of the right trigger.

        :return: The value of the right trigger, ranging from 0.0 to 1.0.
        """
        return self.xbox_controller.getRightTriggerAxis()

    def start_button(self) -> bool:
        """
        Check if the start button is pressed.

        :return: True if the start button is pressed, False otherwise.
        """
        return self.xbox_controller.getStartButton()

    def back_button(self) -> bool:
        """
        Check if the back button is pressed.

        :return: True if the back button is pressed, False otherwise.
        """
        return self.xbox_controller.getBackButton()
