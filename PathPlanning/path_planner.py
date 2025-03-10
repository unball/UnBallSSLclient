# pathplanning/path_planner.py
from threading import Thread, Lock
from queue import Queue
import time
from typing import Dict, List, Tuple
from .astar import AStar, DEBUG_PATH_PLANNING
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
                print(
                    f"Error in path planning: {e}"
                )  # Keep this one - it's an error message

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
            path = self.paths.get(robot_id, [])

        # If field visualization is available and has show_paths attribute
        if hasattr(self.game, "window") and self.game.window:
            try:
                # Try to update path visualization if the method exists
                if hasattr(self.game.window, "update_path_visualization"):
                    self.game.window.update_path_visualization(robot_id, path)
            except Exception as e:
                # Silently ignore visualization errors (don't crash robot control!)
                pass

        return path

    def _plan_path(
        self, robot_id: int, start: Tuple[float, float], goal: Tuple[float, float]
    ):
        """Plan path for a single robot"""
        if DEBUG_PATH_PLANNING:
            print(f"\nPlanning path for robot {robot_id} from {start} to {goal}")

        # Get current vision data
        vision_data = self.game.get_vision_data()
        if not vision_data:
            if DEBUG_PATH_PLANNING:
                print("No vision data available. Cannot plan path.")
            return

        # Create obstacle set from other robots
        obstacles = set()
        for team in ["robotsBlue", "robotsYellow"]:
            for rid, robot in vision_data[team].items():
                if robot["x"] is not None and rid != robot_id:
                    obstacles.add((robot["x"], robot["y"]))
                    if DEBUG_PATH_PLANNING:
                        print(
                            f"Added obstacle at ({robot['x']}, {robot['y']}) from "
                            f"{team} robot {rid}"
                        )

        # Inflate obstacles
        inflation_radius = 0.15  # Robot radius + safety margin
        if DEBUG_PATH_PLANNING:
            print(f"Inflating obstacles with radius {inflation_radius}")
        inflated_obstacles = inflate_obstacles(
            obstacles,
            inflation_radius,
            resolution=0.05,
            field_bounds=self.game.field_bounds,
        )
        if DEBUG_PATH_PLANNING:
            print(f"Total inflated obstacles: {len(inflated_obstacles)}")

        # Verify field bounds
        if not self.game.field_bounds or "x_min" not in self.game.field_bounds:
            # Use default field bounds for SSL-EL (from document)
            self.game.field_bounds = {
                "x_min": -2.25,
                "x_max": 2.25,
                "y_min": -1.5,
                "y_max": 1.5,
            }

            if DEBUG_PATH_PLANNING:
                print("ERROR: Field bounds not properly defined!")
                print(f"Using default field bounds: {self.game.field_bounds}")

        # Check if start and goal are within bounds
        is_start_valid = (
            self.game.field_bounds["x_min"]
            <= start[0]
            <= self.game.field_bounds["x_max"]
            and self.game.field_bounds["y_min"]
            <= start[1]
            <= self.game.field_bounds["y_max"]
        )
        is_goal_valid = (
            self.game.field_bounds["x_min"]
            <= goal[0]
            <= self.game.field_bounds["x_max"]
            and self.game.field_bounds["y_min"]
            <= goal[1]
            <= self.game.field_bounds["y_max"]
        )

        if not is_start_valid:
            # MISSING DEBUG FLAG HERE
            if DEBUG_PATH_PLANNING:
                print(
                    f"WARNING: Start position {start} outside field bounds {self.game.field_bounds}"
                )
                # Adjust start position to be within bounds
                start = (
                    max(
                        min(start[0], self.game.field_bounds["x_max"]),
                        self.game.field_bounds["x_min"],
                    ),
                    max(
                        min(start[1], self.game.field_bounds["y_max"]),
                        self.game.field_bounds["y_min"],
                    ),
                )
                print(f"Adjusted start position to {start}")

        if not is_goal_valid:
            # MISSING DEBUG FLAG HERE
            if DEBUG_PATH_PLANNING:
                print(
                    f"WARNING: Goal position {goal} outside field bounds {self.game.field_bounds}"
                )
                # Adjust goal position to be within bounds
                goal = (
                    max(
                        min(goal[0], self.game.field_bounds["x_max"]),
                        self.game.field_bounds["x_min"],
                    ),
                    max(
                        min(goal[1], self.game.field_bounds["y_max"]),
                        self.game.field_bounds["y_min"],
                    ),
                )
                print(f"Adjusted goal position to {goal}")

        # Plan path
        # MISSING DEBUG FLAG HERE
        if DEBUG_PATH_PLANNING:
            print(
                f"Planning path from {start} to {goal} with field bounds {self.game.field_bounds}"
            )
        path = self.astar.find_path(
            start, goal, self.game.field_bounds, inflated_obstacles
        )

        if not path:
            # MISSING DEBUG FLAG HERE
            if DEBUG_PATH_PLANNING:
                print(f"No path found for robot {robot_id}!")
                # Generate a simple direct path as fallback
                path = [start, goal]
                print(f"Using direct path from {start} to {goal} as fallback")
        else:
            # MISSING DEBUG FLAG HERE
            if DEBUG_PATH_PLANNING:
                print(f"Path found with {len(path)} waypoints for robot {robot_id}")

        # Store result
        with self.paths_lock:
            self.paths[robot_id] = path
