from wpilib import AddressableLED, SmartDashboard


class LED:
    pwm_port: int
    length: int

    def execute(self):
        # Get the mode from the Network Tables
        led_mode = self.get_mode()
        # Change the LED mode based on the value
        if led_mode == "rainbow":
            self.rainbow()
        elif led_mode in self.colors:
            self.set_RGB(self.colors[led_mode])
        else:
            self.turn_off()

    def setup(self) -> None:
        self.colors = {
            "blue": (0, 0, 255),
            "green": (0, 255, 0),
            "red": (255, 0, 0),
            "black": (0, 0, 0),
        }
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

    def set_RGB(self, RGB):
        """
        Set the RGB values for the
        """
        r, g, b = RGB
        for i in range(self.length):
            self.buffer[i].setRGB(r, g, b)
        self.led.setData(self.buffer)
        self.led.start()

    def turn_off(self) -> None:
        """
        Turns the LEDs off.
        """
        self.set_RGB((0, 0, 0))

    # =========================================================================
    # INFORMATIONAL METHODS
    # =========================================================================
    def get_mode(self) -> str:
        return SmartDashboard.getString("LED Mode", "off")
