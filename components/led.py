from wpilib import AddressableLED, SmartDashboard


class LED:
    PWM_PORT: int
    LENGTH: int

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
        elif led_color in self.colors:
            self.set_RGB(self.colors[led_color])
        else:
            self.turn_off()

    def setup(self) -> None:
        self.set_mode("pulse")
        self.set_color("red")
        self.colors = {
            "red": (225, 0, 0),
            "yellow": (255, 127, 0),
            "green": (0, 255, 0),
            "blue": (0, 0, 255),
            "purple": (255, 0, 255),
        }
        self.led = AddressableLED(self.PWM_PORT)
        self.led.setLength(self.LENGTH)
        self.rainbow_first_pixel_hue = 0
        self.knightrider_sign = 1
        self.knightrider_start = 0
        self.pulse_steps = 10
        self.pulse_current_step = 0
        self.pulse_sign = 1

        # Create a buffer with the same length
        self.buffer = [AddressableLED.LEDData() for _ in range(self.LENGTH)]

        # Set the data and start the output
        self._update_leds()

    # =========================================================================
    # CONTROL METHODS
    # =========================================================================

    def knightrider(self, RGB):
        r, g, b = RGB
        for i in range(self.LENGTH):
            if i == self.knightrider_start:
                self.buffer[i].setRGB(r, g, b)
            else:
                self.buffer[i].setRGB(0, 0, 0)
        self.knightrider_start = self.knightrider_start + (self.knightrider_sign * 1)
        if self.knightrider_start == 0:
            self.knightrider_sign = 1

        if self.knightrider_start == self.LENGTH:
            self.knightrider_sign = -1

        self._update_leds()

    def pulse(self, RGB):
        r, g, b = tuple(
            int(RGB[i] + (-RGB[i]) * self.pulse_current_step / self.pulse_steps)
            for i in range(3)
        )
        for i in range(self.LENGTH):
            self.buffer[i].setRGB(r, g, b)

        self.pulse_current_step = self.pulse_current_step + self.pulse_sign
        if self.pulse_current_step == 0 or self.pulse_current_step == self.pulse_steps:
            self.pulse_sign = -1 * self.pulse_sign

        self._update_leds()

    def rainbow(self):
        """
        Produce a rainbow effect
        """
        # For every pixel
        for i in range(self.LENGTH):
            # Calculate the hue
            # Hue is 0-180 in WPILib
            hue = (self.rainbow_first_pixel_hue + (i * 180 / self.LENGTH)) % 180
            # Set the value (HSV: Hue, Saturation, Value/Brightness)
            self.buffer[i].setHSV(int(hue), 255, 128)

        # Increase by to make the rainbow "move"
        self.rainbow_first_pixel_hue += 3
        # Check bounds
        self.rainbow_first_pixel_hue %= 180
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
        for i in range(self.LENGTH):
            self.buffer[i].setRGB(r, g, b)
        self._update_leds()

    def turn_off(self) -> None:
        """
        Turns the LEDs off.
        """
        self.set_RGB((0, 0, 0))

    def _update_leds(self):
        self.led.setData(self.buffer)
        self.led.start()

    # =========================================================================
    # INFORMATIONAL METHODS
    # =========================================================================
    def get_color(self) -> str:
        return SmartDashboard.getString("LED Color", "off")

    def get_mode(self) -> str:
        return SmartDashboard.getString("LED Mode", "off")
