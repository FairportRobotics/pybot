import magicbot


class Elevator:
    elevator_levels: dict
    level = magicbot.tunable(0)
    moving = magicbot.tunable(False)

    def execute(self) -> None:
        pass

    def get_level(self) -> int:
        """
        Get the current level of the elevator.

        :return: The current level of the elevator.
        """
        return self.level

    def move_to(self, level: int) -> None:
        """
        Move the elevator to a specific level.

        :param level: The target level to move the elevator to.
        """
        if level in self.elevator_levels:
            # if not self.moving:
            #   self.moving = True
            self.set_level(level)

    def set_level(self, level: int) -> None:
        """
        Set the elevator to a specific level.

        :param level: The level to set the elevator to.
        """
        self.level = level

    def stop(self) -> None:
        pass
