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

    def get_position(self, robot_id: int) -> Dict[str, float]:
        return {"x": 0.0, "y": 0.0, "theta": 0.0}
