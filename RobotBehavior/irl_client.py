from .robot_base import RobotController
from typing import Dict


class IRLController(RobotController):
    def __init__(self):
        super().__init__()
        self.mode = "IRL"

    def start(self):
        print("Starting IRL controller")

    def stop(self):
        print("Stopping IRL controller")

    def send_global_velocity(self, robot_id: int, vx: float, vy: float, w: float):
        print(f"IRL send_global_velocity - robot {robot_id}: vx={vx}, vy={vy}, w={w}")

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
        """Send command with kick capability"""
        print(
            f"IRL send_command_with_kick - robot {robot_id}: vx={vx}, vy={vy}, w={angular}, kick={kick_speed_x}"
        )

    def kick_flat(self, robot_id: int, kick_speed: float):
        """Send flat kick command"""
        print(f"IRL kick_flat - robot {robot_id}: speed={kick_speed}")

    def kick_chip(self, robot_id: int, kick_speed: float):
        """Send chip kick command"""
        print(f"IRL kick_chip - robot {robot_id}: speed={kick_speed}")

    def get_position(self, robot_id: int) -> Dict[str, float]:
        return {"x": 0.0, "y": 0.0, "theta": 0.0}
