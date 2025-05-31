from pyfrc.physics import drivetrains

class PhysicsEngine:
    def __init__(self, physics_controller):
        self.physics_controller = physics_controller

        self.drivetrain = drivetrains.TankModel.theory(
            motor_left=2,
            motor_right=2,
            robot_mass=110,       # in pounds
            gear_ratio=10.71,
            wheelbase=2.0,        # in feet
            robot_width=3.0,      # in feet
            robot_length=3.0,     # in feet
            wheel_diameter=0.5    # in feet (6-inch wheels)
        )

    def update_sim(self, hal_data, now, tm_diff):
        # Get left and right motor outputs (from arcadeDrive)
        l_motor = hal_data["pwm"][0]["value"]  # Left motor (PWM 0)
        r_motor = hal_data["pwm"][1]["value"]  # Right motor (PWM 1)

        # Calculate position update
        x, y, angle = self.drivetrain.calculate(l_motor, r_motor, tm_diff)
        self.physics_controller.drive.set_pose(x, y, angle)
