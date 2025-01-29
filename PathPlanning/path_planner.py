# pathplanning/path_planner.py
from threading import Thread, Lock
from queue import Queue
import time
from typing import Dict, List, Tuple

from .astar import AStar
from .utils import inflate_obstacles


class PathPlanner(Thread):
    def __init__(self, game):
        super().__init__()
        self.daemon = True  # Make thread daemon so it exits with main
        self.game = game
        self.astar = AStar()
        self.running = False
        self.planning_queue = Queue()
        self.paths: Dict[int, List[Tuple[float, float]]] = {}
        self.paths_lock = Lock()

    def run(self):
        """Main planning loop"""
        self.running = True
        while self.running:
            try:
                if not self.planning_queue.empty():
                    robot_id, start, goal = self.planning_queue.get_nowait()
                    self._plan_path(robot_id, start, goal)
                time.sleep(0.01)
            except Exception as e:
                print(f"Error in path planning: {e}")

    def stop(self):
        """Stop the planning thread"""
        if not self.running:
            return
        self.running = False
        # Clear queue
        while not self.planning_queue.empty():
            self.planning_queue.get_nowait()
        # Wait for thread to finish
        if self.is_alive():
            self.join(timeout=1.0)

    def request_path(
        self, robot_id: int, start: Tuple[float, float], goal: Tuple[float, float]
    ):
        """Request a new path to be planned"""
        self.planning_queue.put((robot_id, start, goal))

    def get_path(self, robot_id: int) -> List[Tuple[float, float]]:
        """Get the currently planned path for a robot"""
        with self.paths_lock:
            return self.paths.get(robot_id, [])

    def _plan_path(
        self, robot_id: int, start: Tuple[float, float], goal: Tuple[float, float]
    ):
        """Plan path for a single robot"""
        # Get current vision data
        vision_data = self.game.get_vision_data()
        if not vision_data:
            return

        # Create obstacle set from other robots
        obstacles = set()
        for team in ["robotsBlue", "robotsYellow"]:
            for rid, robot in vision_data[team].items():
                if robot["x"] is not None and rid != robot_id:
                    obstacles.add((robot["x"], robot["y"]))

        # Inflate obstacles
        inflation_radius = 0.15  # Robot radius + safety margin
        inflated_obstacles = inflate_obstacles(
            obstacles, inflation_radius, resolution=0.05
        )

        # Plan path
        path = self.astar.find_path(
            start, goal, self.game.field_bounds, inflated_obstacles
        )

        # Store result
        with self.paths_lock:
            self.paths[robot_id] = path
