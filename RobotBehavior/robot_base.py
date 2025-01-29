from abc import ABC, abstractmethod
from typing import Dict, Optional, Tuple


class RobotController(ABC):
    def __init__(self):
        self.mode = "base"

    @abstractmethod
    def send_global_velocity(self, robot_id: int, vx: float, vy: float, w: float):
        pass

    @abstractmethod
    def get_position(self, robot_id: int) -> Dict[str, float]:
        pass

    @abstractmethod
    def start(self):
        pass

    @abstractmethod
    def stop(self):
        pass
