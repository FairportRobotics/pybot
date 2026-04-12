from abc import ABC, abstractmethod


class MotorComponent(ABC):
    """Base interface for all motor components."""

    @abstractmethod
    def set_output(self, output: float) -> None:
        pass

    @abstractmethod
    def get_velocity(self) -> float:
        pass

    @abstractmethod
    def get_position(self) -> float:
        pass

    @abstractmethod
    def execute(self) -> None:
        pass
