import threading
import queue
import time
import numpy as np
from typing import Dict, List, Tuple, Set, Optional
import math


class PathPlanner(threading.Thread):
    """
    A threaded path planner that uses A* algorithm to find paths for robots
    """

    def __init__(self, game):
        """
        Initialize the path planner thread

        Args:
            game: The game instance with access to vision data
        """
        super().__init__()
        self.daemon = True  # Daemon threads exit when the main program exits
        self.game = game

        # Thread control
        self.running = False
        self.planning_lock = threading.Lock()
        self.paths_lock = threading.Lock()

        # Planning queue and results storage
        self.planning_queue = queue.Queue()
        self.paths = {}  # robot_id -> path

        # Path planning parameters
        self.resolution = 0.05  # Grid resolution in meters
        self.inflation_radius = 0.15  # Robot radius + safety margin

        # Debug
        self.debug = game.debug.get("path_planning", False)

    def run(self):
        """Main thread loop to process planning requests"""
        self.running = True

        while self.running:
            try:
                # Get next planning request from queue (non-blocking)
                try:
                    robot_id, start, goal = self.planning_queue.get(block=False)

                    # Plan path for this robot
                    if self.debug:
                        print(
                            f"Planning path for robot {robot_id} from {start} to {goal}"
                        )

                    path = self._plan_path(robot_id, start, goal)

                    # Store result
                    with self.paths_lock:
                        self.paths[robot_id] = path

                    # Mark task as done
                    self.planning_queue.task_done()

                except queue.Empty:
                    # No planning requests, sleep briefly
                    time.sleep(0.005)  # 5ms

            except Exception as e:
                if self.debug:
                    print(f"Error in path planning thread: {e}")
                time.sleep(0.1)  # Recover from errors

    def stop(self):
        """Stop the planning thread safely"""
        self.running = False
        # Clear queue
        while not self.planning_queue.empty():
            try:
                self.planning_queue.get_nowait()
                self.planning_queue.task_done()
            except queue.Empty:
                break

        # Wait for thread to finish
        if self.is_alive():
            self.join(timeout=1.0)

    def request_path(
        self, robot_id: int, start: Tuple[float, float], goal: Tuple[float, float]
    ):
        """
        Request a path to be planned

        Args:
            robot_id: ID of the robot
            start: Starting position (x, y)
            goal: Goal position (x, y)
        """
        # Validate input
        if not self._is_valid_position(start) or not self._is_valid_position(goal):
            if self.debug:
                print(f"Invalid start or goal position: {start} -> {goal}")
            return

        # Add to planning queue
        self.planning_queue.put((robot_id, start, goal))

    def get_path(self, robot_id: int) -> List[Tuple[float, float]]:
        """
        Get the currently planned path for a robot

        Args:
            robot_id: ID of the robot

        Returns:
            List of (x, y) waypoints or empty list if no path
        """
        with self.paths_lock:
            return self.paths.get(robot_id, [])

    def _is_valid_position(self, pos: Tuple[float, float]) -> bool:
        """
        Check if a position is valid (within field bounds and not None)

        Args:
            pos: Position to check (x, y)

        Returns:
            True if position is valid
        """
        if pos is None or len(pos) != 2:
            return False

        x, y = pos
        if x is None or y is None or math.isnan(x) or math.isnan(y):
            return False

        # Check if within field bounds with some margin
        bounds = self.game.field_bounds
        margin = 0.1  # 10cm margin

        return (
            bounds["x_min"] - margin <= x <= bounds["x_max"] + margin
            and bounds["y_min"] - margin <= y <= bounds["y_max"] + margin
        )

    def _plan_path(
        self, robot_id: int, start: Tuple[float, float], goal: Tuple[float, float]
    ) -> List[Tuple[float, float]]:
        """Plan a path using A* algorithm"""
        # Start timing
        start_time = time.time()

        # Get vision data to extract obstacles
        vision_data = self.game.get_vision_data()
        if not vision_data:
            # No vision data, return direct path
            return [start, goal]

        # Extract obstacles from other robots
        obstacles = self._get_obstacles(vision_data, robot_id)

        # If no obstacles or close enough, return direct path
        if not obstacles or self._euclidean_distance(start, goal) < 0.3:
            if self.debug:
                print(
                    f"Direct path for robot {robot_id}: {len(obstacles)} obstacles, distance={self._euclidean_distance(start, goal):.2f}m"
                )
            return [start, goal]

        # Run A* algorithm
        path = self._astar(start, goal, obstacles)

        # If path couldn't be found, return direct path
        if not path:
            if self.debug:
                print(f"No path found for robot {robot_id}, using direct path")
            return [start, goal]

        # Post-process path to smooth it
        smoothed_path = self._smooth_path(path)

        # Calculate and log planning time
        planning_time = time.time() - start_time
        if self.debug:
            print(
                f"Path planning for robot {robot_id} took {planning_time*1000:.2f}ms: {len(path)} points → {len(smoothed_path)} after smoothing"
            )

        return smoothed_path

    def _get_obstacles(
        self, vision_data: Dict, robot_id: int
    ) -> Set[Tuple[float, float]]:
        """
        Extract obstacles from vision data

        Args:
            vision_data: Vision data from SSL-Vision
            robot_id: ID of the robot for which we're planning

        Returns:
            Set of (x, y) obstacle positions
        """
        obstacles = set()

        # Extract all robots except the one we're planning for
        for team in ["robotsBlue", "robotsYellow"]:
            for rid, robot in vision_data.get(team, {}).items():
                # Skip if robot is not detected or is the planning robot
                if robot["x"] is None or int(rid) == robot_id:
                    continue

                # Add robot position to obstacles
                obstacles.add((robot["x"], robot["y"]))

        # Inflate obstacles
        inflated_obstacles = self._inflate_obstacles(obstacles)

        return inflated_obstacles

    def _inflate_obstacles(
        self, obstacles: Set[Tuple[float, float]]
    ) -> Set[Tuple[float, float]]:
        """
        Inflate obstacles by the robot radius plus safety margin

        Args:
            obstacles: Original obstacle positions

        Returns:
            Set of inflated obstacle positions
        """
        if not obstacles:
            return set()

        # Convert to numpy array for efficient operations
        obstacle_array = np.array(list(obstacles))

        # Create a grid of points around each obstacle
        steps = int(self.inflation_radius / self.resolution)
        inflated = set()

        for x, y in obstacle_array:
            for dx in range(-steps, steps + 1):
                for dy in range(-steps, steps + 1):
                    # Check if point is within inflation radius
                    px = x + dx * self.resolution
                    py = y + dy * self.resolution
                    distance = math.sqrt(dx**2 + dy**2) * self.resolution

                    if distance <= self.inflation_radius:
                        # Add to inflated obstacles set
                        inflated.add((round(px, 3), round(py, 3)))

        return inflated

    def _astar(
        self,
        start: Tuple[float, float],
        goal: Tuple[float, float],
        obstacles: Set[Tuple[float, float]],
    ) -> List[Tuple[float, float]]:
        """
        A* pathfinding algorithm

        Args:
            start: Starting position (x, y)
            goal: Goal position (x, y)
            obstacles: Set of obstacle positions

        Returns:
            List of (x, y) waypoints
        """
        # A* implementation here

        class Node:
            def __init__(self, x, y):
                self.x = x
                self.y = y
                self.g = float("inf")  # Cost from start to this node
                self.h = float("inf")  # Heuristic (estimated cost to goal)
                self.f = float("inf")  # Total cost (g + h)
                self.parent = None

            def __eq__(self, other):
                return self.x == other.x and self.y == other.y

            def __lt__(self, other):
                return self.f < other.f

            def __hash__(self):
                return hash((self.x, self.y))

        # Round start and goal to grid resolution
        start_x = round(start[0] / self.resolution) * self.resolution
        start_y = round(start[1] / self.resolution) * self.resolution
        goal_x = round(goal[0] / self.resolution) * self.resolution
        goal_y = round(goal[1] / self.resolution) * self.resolution

        # Create start and goal nodes
        start_node = Node(start_x, start_y)
        goal_node = Node(goal_x, goal_y)

        # Initialize costs
        start_node.g = 0
        start_node.h = self._heuristic(start_node, goal_node)
        start_node.f = start_node.g + start_node.h

        # Initialize open and closed sets
        open_set = [start_node]
        closed_set = set()

        # Define movement directions (8-connected grid)
        directions = [
            (1, 0),
            (-1, 0),
            (0, 1),
            (0, -1),  # Cardinal
            (1, 1),
            (-1, 1),
            (1, -1),
            (-1, -1),  # Diagonal
        ]

        # Main loop
        while open_set:
            # Get node with lowest f-cost
            current = min(open_set, key=lambda n: n.f)

            # Check if goal reached
            if (
                self._euclidean_distance(
                    (current.x, current.y), (goal_node.x, goal_node.y)
                )
                < self.resolution
            ):
                # Reconstruct path
                path = []
                while current:
                    path.append((current.x, current.y))
                    current = current.parent
                return path[::-1]  # Reverse path (start to goal)

            # Move current from open to closed set
            open_set.remove(current)
            closed_set.add((current.x, current.y))

            # Check neighbors
            for dx, dy in directions:
                # Calculate neighbor position
                nx = current.x + dx * self.resolution
                ny = current.y + dy * self.resolution

                # Check if in closed set or is obstacle
                if (nx, ny) in closed_set or self._is_obstacle((nx, ny), obstacles):
                    continue

                # Create neighbor node
                neighbor = Node(nx, ny)

                # Calculate g-cost (cost from start to neighbor through current)
                move_cost = (
                    self.resolution
                    if abs(dx) + abs(dy) == 1
                    else 1.414 * self.resolution
                )
                tentative_g = current.g + move_cost

                # Check if neighbor is in open set
                existing = next((n for n in open_set if n == neighbor), None)

                if existing is None:
                    # Add to open set
                    neighbor.g = tentative_g
                    neighbor.h = self._heuristic(neighbor, goal_node)
                    neighbor.f = neighbor.g + neighbor.h
                    neighbor.parent = current
                    open_set.append(neighbor)
                elif tentative_g < existing.g:
                    # Update existing neighbor
                    existing.g = tentative_g
                    existing.f = existing.g + existing.h
                    existing.parent = current

        # No path found
        return []

    def _smooth_path(
        self, path: List[Tuple[float, float]]
    ) -> List[Tuple[float, float]]:
        """
        Smooth the path by removing unnecessary waypoints

        Args:
            path: Original path

        Returns:
            Smoothed path
        """
        if len(path) <= 2:
            return path

        # Simple smoothing: keep only essential waypoints
        smoothed = [path[0]]

        for i in range(1, len(path) - 1):
            prev_vec = (path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1])
            next_vec = (path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1])

            # Calculate angle between vectors
            angle = self._angle_between(prev_vec, next_vec)

            # Keep waypoint if angle is significant
            if abs(angle) > 0.3:  # ~17 degrees
                smoothed.append(path[i])

        smoothed.append(path[-1])
        return smoothed

    def _heuristic(self, node: "Node", goal: "Node") -> float:
        """
        Heuristic function (Manhattan distance)

        Args:
            node: Current node
            goal: Goal node

        Returns:
            Heuristic cost estimate
        """
        return abs(node.x - goal.x) + abs(node.y - goal.y)

    def _is_obstacle(
        self, pos: Tuple[float, float], obstacles: Set[Tuple[float, float]]
    ) -> bool:
        """
        Check if position is an obstacle

        Args:
            pos: Position to check
            obstacles: Set of obstacle positions

        Returns:
            True if position is an obstacle
        """
        # Check exact match
        rounded_pos = (round(pos[0], 3), round(pos[1], 3))
        if rounded_pos in obstacles:
            return True

        # Check nearby obstacles (within resolution)
        for ox, oy in obstacles:
            if self._euclidean_distance(pos, (ox, oy)) < self.resolution:
                return True

        return False

    def _euclidean_distance(
        self, p1: Tuple[float, float], p2: Tuple[float, float]
    ) -> float:
        """
        Calculate Euclidean distance between two points

        Args:
            p1: First point
            p2: Second point

        Returns:
            Euclidean distance
        """
        return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

    def _angle_between(self, v1: Tuple[float, float], v2: Tuple[float, float]) -> float:
        """
        Calculate angle between two vectors

        Args:
            v1: First vector
            v2: Second vector

        Returns:
            Angle in radians
        """
        # Calculate dot product
        dot = v1[0] * v2[0] + v1[1] * v2[1]

        # Calculate magnitudes
        mag1 = math.sqrt(v1[0] ** 2 + v1[1] ** 2)
        mag2 = math.sqrt(v2[0] ** 2 + v2[1] ** 2)

        # Calculate angle
        cos_angle = dot / (mag1 * mag2) if mag1 * mag2 > 0 else 0

        # Clamp value to valid range
        cos_angle = max(-1.0, min(1.0, cos_angle))

        return math.acos(cos_angle)
