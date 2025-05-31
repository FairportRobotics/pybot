import magicbot
import time


class Elevator:
    elevator_levels: dict
    current_level = magicbot.tunable(0)
    target_level = magicbot.tunable(0)
    is_not_moving = magicbot.tunable(True)

    def execute(self) -> None:
        """
        Execute the elevator movement logic.
        This method is called periodically to check if the elevator needs to move to the target level.
        """
        if self.target_level != self.current_level:
            self.is_not_moving = False
            # Simulate moving to the target level
            time.sleep(1.5)
            self.set_current_level(self.target_level)
            self.is_not_moving = True

    def get_current_level(self) -> int:
        """
        Get the current level of the elevator.

        :return: The current level of the elevator.
        """
        return self.current_level

    def move_to(self, target_level: int) -> None:
        """
        Move the elevator to a specific level.

        :param target_level: The target level to move the elevator to.
        """
        # Change the target if the level is valid and the elevator is not currently moving
        if target_level in self.elevator_levels and self.is_not_moving:
            self.set_target_level(target_level)

    def set_current_level(self, level: int) -> None:
        """
        Set the current level of the elevator.

        :param level: The level to elevator is currently at.
        """
        self.current_level = level

    def set_target_level(self, level: int) -> None:
        """
        Set the target level for the elevator.

        :param level: The level you want the elevator to move to.
        """
        self.target_level = level

    def stop(self) -> None:
        # TODO: implement stop functionality
        pass
