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
        return self.client.send_command_with_kick(
            robot_id, vx, vy, angular, kick_speed_x, kick_speed_z, dribbler_speed
        )

    def kick_flat(self, robot_id: int, kick_speed: float):
        """Send flat kick command"""
        return self.client.kick_flat(robot_id, kick_speed)

    def kick_chip(self, robot_id: int, kick_speed: float):
        """Send chip kick command"""
        return self.client.kick_chip(robot_id, kick_speed)

    def get_position(self, robot_id: int) -> Dict[str, float]:
        return self.client.get_position(robot_id)
