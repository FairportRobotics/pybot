from pyfrc.physics.tankmodel import TankModel
from pyfrc.physics.units import units
from pyfrc.physics import motor_cfgs


class PhysicsEngine:
    def __init__(self, physics_controller):
        self.physics_controller = physics_controller

        # create a tank model
        self.tank_model = TankModel.theory(
            motor_config=motor_cfgs.MOTOR_CFG_CIM,
            robot_mass=(115 * units.lbs),
            gearing=10.71,
            robot_length=(28 * units.inch),
            robot_width=(28 * units.inch),
        )

        # initial position
        self.x = 0
        self.y = 0
        self.heading = 0

    def update_sim(self, now, tm_diff):
        """
        # update the motors
        l_motor = self.physics_controller.get_motor(1)
        r_motor = self.physics_controller.get_motor(2)

        # get the robot velocity
        l_vel, r_vel = self.tank_model.get_wheel_velocities(l_motor, r_motor)
        velocity, angular_velocity = self.tank_model.get_vector(l_motor, r_motor, tm_diff)

        # update the position
        self.x, self.y, self.heading = self.physics_controller.move_robot(self.x, self.y, self.heading, velocity, angular_velocity, tm_diff)
        """
        pass
