from wpilib import AddressableLED, SmartDashboard


class LED:
    pwm_port: int
    length: int

    def execute(self):
        # Get the mode from the Network Tables
        led_mode = self.get_mode()
        # Change the LED mode based on the value
        if led_mode == "blue":
            self.blue()
        elif led_mode == "green":
            self.green()
        elif led_mode == "red":
            self.red()
        elif led_mode == "rainbow":
            self.rainbow()
        else:
            self.turn_off()

    def setup(self) -> None:
        self.led = AddressableLED(self.pwm_port)
        self.led.setLength(self.length)
        self.rainbow_first_pixel_hue = 0

        # Create a buffer with the same length
        self.buffer = [AddressableLED.LEDData() for _ in range(self.length)]

        # Set the data and start the output
        self.led.setData(self.buffer)
        self.led.start()

    # =========================================================================
    # CONTROL METHODS
    # =========================================================================

    def blue(self):
        """
        Turn the LEDs blue
        """
        self.set_RGB(0, 0, 255)

    def green(self):
        """
        Turn the LEDs geen
        """
        self.set_RGB(0, 255, 0)

    def red(self):
        """
        Turn the LEDs red
        """
        self.set_RGB(255, 0, 0)

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
        self.led.setData(self.buffer)
        self.led.start()

    def set_mode(self, mode: str) -> None:
        """
        Write the LED mode to Network Tables
        """
        SmartDashboard.putString("LED Mode", mode)

    def set_RGB(self, r, g, b):
        """
        Set the RGB values for the
        """
        for i in range(self.length):
            self.buffer[i].setRGB(r, g, b)
        self.led.setData(self.buffer)
        self.led.start()

    def turn_off(self) -> None:
        """
        Turns the LEDs off.
        """
        self.set_RGB(0, 0, 0)

    # =========================================================================
    # INFORMATIONAL METHODS
    # =========================================================================
    def get_mode(self) -> str:
        return SmartDashboard.getString("LED Mode", "off")
