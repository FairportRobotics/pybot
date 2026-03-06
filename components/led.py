from wpilib import AddressableLED


class LED:
    pwm_port: int
    length: int

    def execute(self):
        pass

    def setup(self) -> None:
        self.led = AddressableLED(self.pwm_port)
        self.led.setLength(self.length)
        self.rainbow_first_pixel_hue = 0

        # Create a buffer with the same length
        self.buffer =[AddressableLED.LEDData() for _ in range(self.length)]

        # Set the data and start the output
        self.led.setData(self.buffer)
        self.led.start()

    # =========================================================================
    # CONTROL METHODS
    # =========================================================================

    def _set_RGB(self, r,g,b):
        for i in range(self.length):
            self.buffer[i].setRGB(r, g, b) 
        self.led.setData(self.buffer)
        self.led.start()


    def green(self):
        self._set_RGB(0, 255, 0) # R=0, G=255, B=0

    def red(self):
        self._set_RGB(255, 0, 0) # R=255, G=0, B=0

    def rainbow(self):
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


    def turn_off(self) -> None:
        """
        Turn the LED off.
        """
        self._set_RGB(0,0,0)
    