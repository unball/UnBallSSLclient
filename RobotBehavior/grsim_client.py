from .robot_base import RobotController
from SimulationGrSim.RobotControlClient_threaded import ThreadedRobotControlClient
from typing import Dict


class GrSimController(RobotController):
    def __init__(self, team_port: int):
        super().__init__()
        self.mode = "grSim"
        self.client = ThreadedRobotControlClient(team_port=team_port)

    def start(self):
        self.client.start()

    def stop(self):
        self.client.stop()

    def send_global_velocity(self, robot_id: int, vx: float, vy: float, w: float):
        return self.client.send_global_velocity(robot_id, vx, vy, w)

    def get_position(self, robot_id: int) -> Dict[str, float]:
        return self.client.get_position(robot_id)
