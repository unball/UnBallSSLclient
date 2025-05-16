from abc import ABC, abstractmethod
from typing import Dict, Optional, Tuple


class RobotController(ABC):
    def __init__(self):
        self.mode = "base"

    @abstractmethod
    def send_global_velocity(self, robot_id: int, vx: float, vy: float, w: float):
        pass

    def send_command_with_kick(
        self,
        robot_id: int,
        vx: float,
        vy: float,
        angular: float,
        kick_speed_x: float,
        kick_speed_z: float = 0.0,
        dribbler_speed: float = 0.0,
    ):
        """Send command with kick capability - default implementation"""
        # Default: just send the movement command
        return self.send_global_velocity(robot_id, vx, vy, angular)

    def kick_flat(self, robot_id: int, kick_speed: float):
        """Send flat kick command - default implementation"""
        return self.send_command_with_kick(robot_id, 0, 0, 0, kick_speed, 0.0, 0.0)

    def kick_chip(self, robot_id: int, kick_speed: float):
        """Send chip kick command - default implementation"""
        return self.send_command_with_kick(
            robot_id, 0, 0, 0, kick_speed, kick_speed, 0.0
        )

    @abstractmethod
    def get_position(self, robot_id: int) -> Dict[str, float]:
        pass

    @abstractmethod
    def start(self):
        pass

    @abstractmethod
    def stop(self):
        pass
