from wpilib import AddressableLED, SmartDashboard
import random


class LED:
    pwm_port: int
    length: int
    default_color: str = "off"
    default_mode: str = "off"

    def execute(self):
        # Get the mode from the Network Tables
        led_color = self.get_color()
        led_mode = self.get_mode()

        # Change the LED mode based on the value
        if led_mode == "rainbow":
            self.rainbow()
        elif led_mode == "knightrider":
            self.knightrider(self.colors[led_color])
        elif led_mode == "pulse":
            self.pulse(self.colors[led_color])
        elif led_mode == "sparkle":
            self.sparkle(self.colors[led_color])
        elif led_mode == "solid" and led_color in self.colors:
            self.set_RGB(self.colors[led_color])
        else:
            self.turn_off()

    def setup(self) -> None:
        # Set the default mode and colors
        self.set_mode(self.default_mode)
        self.set_color(self.default_color)

        # Define colors in RGB values
        self.colors = {
            "red": (225, 0, 0),
            "yellow": (255, 127, 0),
            "green": (0, 255, 0),
            "blue": (0, 0, 255),
            "purple": (255, 0, 255),
        }

        # Set up the LEDs
        self.led = AddressableLED(self.pwm_port)
        self.led.setLength(self.length)

        # Define variables for the LED modes
        self.rainbow_first_pixel_hue = 0
        self.knightrider_sign = 1
        self.knightrider_light_up = 0
        self.pulse_steps = 10
        self.pulse_current_step = 0
        self.pulse_sign = 1

        # Create a buffer
        self.buffer = [AddressableLED.LEDData() for _ in range(self.length)]

        # Set the data and start the output
        self._update_leds()

    # =========================================================================
    # CONTROL METHODS
    # =========================================================================

    def knightrider(self, RGB):
        """
        Create a knight rider effect for a color
        """
        r, g, b = RGB
        for i in range(self.length):
            # Turn on a single LED
            if i == self.knightrider_light_up:
                self.buffer[i].setRGB(r, g, b)
            # Turn off all others
            else:
                self.buffer[i].setRGB(0, 0, 0)

        # Change the LED to light up
        self.knightrider_light_up = self.knightrider_light_up + (
            self.knightrider_sign * 1
        )

        # Check if we need to flip the sign
        if self.knightrider_light_up == 0 or self.knightrider_light_up == self.length:
            self.knightrider_sign = -1 * self.knightrider_sign

        self._update_leds()

    def pulse(self, RGB):
        """
        Create a pulsing effect for a color
        """
        # Create the RGB values based on which step we currently are in
        r, g, b = tuple(
            int(RGB[i] + (-RGB[i]) * self.pulse_current_step / self.pulse_steps)
            for i in range(3)
        )

        # Apply it to all the LEDs
        for i in range(self.length):
            self.buffer[i].setRGB(r, g, b)

        # Increment/decrement the current step
        self.pulse_current_step = self.pulse_current_step + self.pulse_sign

        # Check if we need to flip the sign
        if self.pulse_current_step == 0 or self.pulse_current_step == self.pulse_steps:
            self.pulse_sign = -1 * self.pulse_sign

        self._update_leds()

    def rainbow(self):
        """
        Produce a rainbow effect
        """
        # For every pixel
        for i in range(self.length):
            # Calculate the hue
            # Hue is 0-180 in WPILib
            hue = (self.rainbow_first_pixel_hue + (i * 180 / self.length)) % 180
            # Set the value (HSV: Hue, Saturation, Value/Brightness)
            self.buffer[i].setHSV(int(hue), 255, 128)

        # Increase by to make the rainbow "move"
        self.rainbow_first_pixel_hue += 3

        # Check bounds
        self.rainbow_first_pixel_hue %= 180

        self._update_leds()

    def sparkle(self, RGB, chance=0.1):
        """
        Create a sparkling effect for a color
        """
        r, g, b = RGB
        for i in range(self.length):
            if random.random() < chance:
                self.buffer[i].setRGB(r, g, b)
            else:
                self.buffer[i].setRGB(0, 0, 0)

        self._update_leds()

    def set_color(self, color: str) -> None:
        """
        Write the LED color to Network Tables
        """
        SmartDashboard.putString("LED Color", color)

    def set_mode(self, mode: str) -> None:
        """
        Write the LED mode to Network Tables
        """
        SmartDashboard.putString("LED Mode", mode)

    def set_RGB(self, RGB):
        """
        Set the RGB values for the
        """
        r, g, b = RGB
        for i in range(self.length):
            self.buffer[i].setRGB(r, g, b)
        self._update_leds()

    def turn_off(self) -> None:
        """
        Turns the LEDs off.
        """
        self.set_RGB((0, 0, 0))

    def _update_leds(self):
        """
        Update the LED data and start the output
        """
        self.led.setData(self.buffer)
        self.led.start()

    # =========================================================================
    # INFORMATIONAL METHODS
    # =========================================================================
    def get_color(self) -> str:
        return SmartDashboard.getString("LED Color", "off")

    def get_mode(self) -> str:
        return SmartDashboard.getString("LED Mode", "off")
