"""Xbox controller abstraction.

Wraps a WPILib ``XboxController`` and exposes all inputs as clean properties
with deadband filtering and optional axis inversion.

Design goals
------------
* **Generalised** — the same class works for both driver and operator
  controllers by passing a different port number.
* **Inversion flag** — ``invert_axes=True`` (default) negates the joystick
  values so that pushing the stick forward produces a positive ``vx``.  This
  matches the FRC convention where the Y-axis of most Xbox controllers reads
  negative when pushed forward.
* **Single source of truth** — all button/axis mappings live here so that
  ``robot.py`` never touches raw axis indices.
"""

from __future__ import annotations

import math

import wpilib

import constants


class XboxController:
    """Thin, readable wrapper around ``wpilib.XboxController``.

    Instantiation
    -------------
    Create this in ``robot.createObjects()`` and store as ``self.driver_controller``::

        self.driver_controller = XboxController(
            port=constants.DRIVER_CONTROLLER_PORT,
            invert_axes=True,
        )

    To create a second operator controller, instantiate with a different port::

        self.operator_controller = XboxController(
            port=constants.OPERATOR_CONTROLLER_PORT,
            invert_axes=True,
        )

    Parameters
    ----------
    port:
        USB port index on the Driver Station (0 = driver, 1 = operator).
    invert_axes:
        When ``True`` (default), joystick Y-axes are negated so that pushing
        forward produces a positive value.
    """

    def __init__(
        self,
        port: int = constants.DRIVER_CONTROLLER_PORT,
        invert_axes: bool = True,
    ) -> None:
        self._controller = wpilib.XboxController(port)
        self.invert_axes = invert_axes

    # ------------------------------------------------------------------
    # Axes — translation and rotation
    # ------------------------------------------------------------------

    @property
    def vx(self) -> float:
        """Forward/backward translation [-1, 1].

        Positive → move towards opposing alliance wall.
        """
        raw = self._controller.getLeftY()
        return self._apply_deadband(self._maybe_invert(raw))

    @property
    def vy(self) -> float:
        """Left/right strafe [-1, 1].

        Positive → strafe left.
        """
        raw = self._controller.getLeftX()
        return self._apply_deadband(self._maybe_invert(raw))

    @property
    def omega(self) -> float:
        """Rotation rate [-1, 1].

        Positive → rotate counter-clockwise (left).
        """
        raw = self._controller.getRightX()
        return self._apply_deadband(self._maybe_invert(raw))

    @property
    def right_trigger(self) -> float:
        """Right trigger axis [0, 1].  Typically used to fire the shooter."""
        return self._controller.getRightTriggerAxis()

    @property
    def left_trigger(self) -> float:
        """Left trigger axis [0, 1].  Typically used to run the intake."""
        return self._controller.getLeftTriggerAxis()

    # ------------------------------------------------------------------
    # Buttons
    # ------------------------------------------------------------------

    @property
    def a_button(self) -> bool:
        """A button — pressed state."""
        return self._controller.getAButton()

    @property
    def b_button(self) -> bool:
        """B button — pressed state."""
        return self._controller.getBButton()

    @property
    def x_button(self) -> bool:
        """X button — pressed state."""
        return self._controller.getXButton()

    @property
    def y_button(self) -> bool:
        """Y button — pressed state."""
        return self._controller.getYButton()

    @property
    def left_bumper(self) -> bool:
        """Left bumper — pressed state."""
        return self._controller.getLeftBumperButton()

    @property
    def right_bumper(self) -> bool:
        """Right bumper — pressed state."""
        return self._controller.getRightBumperButton()

    @property
    def start_button(self) -> bool:
        """Start (hamburger) button — pressed state."""
        return self._controller.getStartButton()

    @property
    def back_button(self) -> bool:
        """Back (view) button — pressed state."""
        return self._controller.getBackButton()

    @property
    def left_stick_button(self) -> bool:
        """Left stick click — pressed state."""
        return self._controller.getLeftStickButton()

    @property
    def right_stick_button(self) -> bool:
        """Right stick click — pressed state."""
        return self._controller.getRightStickButton()

    # Rising-edge (pressed this loop) variants for single-shot actions.

    @property
    def a_button_pressed(self) -> bool:
        """``True`` only on the loop the A button is first pressed."""
        return self._controller.getAButtonPressed()

    @property
    def b_button_pressed(self) -> bool:
        """``True`` only on the loop the B button is first pressed."""
        return self._controller.getBButtonPressed()

    @property
    def x_button_pressed(self) -> bool:
        """``True`` only on the loop the X button is first pressed."""
        return self._controller.getXButtonPressed()

    @property
    def y_button_pressed(self) -> bool:
        """``True`` only on the loop the Y button is first pressed."""
        return self._controller.getYButtonPressed()

    @property
    def start_button_pressed(self) -> bool:
        """``True`` only on the loop the Start button is first pressed."""
        return self._controller.getStartButtonPressed()

    @property
    def back_button_pressed(self) -> bool:
        """``True`` only on the loop the Back button is first pressed."""
        return self._controller.getBackButtonPressed()

    @property
    def left_bumper_pressed(self) -> bool:
        """``True`` only on the loop the left bumper is first pressed."""
        return self._controller.getLeftBumperButtonPressed()

    @property
    def right_bumper_pressed(self) -> bool:
        """``True`` only on the loop the right bumper is first pressed."""
        return self._controller.getRightBumperButtonPressed()

    # ------------------------------------------------------------------
    # D-Pad (POV)
    # ------------------------------------------------------------------

    def _pov(self) -> int:
        """Raw POV angle in degrees, or -1 when not pressed."""
        return self._controller.getPOV()

    @property
    def dpad_up(self) -> bool:
        """D-Pad up (0°)."""
        return self._pov() == 0

    @property
    def dpad_right(self) -> bool:
        """D-Pad right (90°)."""
        return self._pov() == 90

    @property
    def dpad_down(self) -> bool:
        """D-Pad down (180°)."""
        return self._pov() == 180

    @property
    def dpad_left(self) -> bool:
        """D-Pad left (270°)."""
        return self._pov() == 270

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _maybe_invert(self, value: float) -> float:
        """Negate ``value`` when ``self.invert_axes`` is ``True``."""
        return -value if self.invert_axes else value

    @staticmethod
    def _apply_deadband(value: float) -> float:
        """Return 0.0 when ``|value|`` is within the configured deadband."""
        if math.fabs(value) < constants.JOYSTICK_DEADBAND:
            return 0.0
        return value
