# pathplanning/astar.py
from dataclasses import dataclass
import heapq
import numpy as np
from typing import List, Tuple, Set

# Set this to False to disable debug prints, True to enable them
DEBUG_PATH_PLANNING = False


@dataclass
class Node:
    x: float
    y: float
    g_cost: float = float("inf")
    h_cost: float = float("inf")
    parent: "Node" = None

    @property
    def f_cost(self) -> float:
        return self.g_cost + self.h_cost

    def __lt__(self, other):
        return self.f_cost < other.f_cost

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y


class AStar:
    def __init__(self, resolution=0.05):
        self.resolution = resolution  # Grid resolution in meters
        self.movements = [
            (1, 0),
            (-1, 0),
            (0, 1),
            (0, -1),  # Cardinal directions
            (1, 1),
            (-1, 1),
            (1, -1),
            (-1, -1),  # Diagonals
        ]

    def heuristic(self, node: Node, goal_node: Node) -> float:
        """Manhattan distance heuristic"""
        return abs(node.x - goal_node.x) + abs(node.y - goal_node.y)

    def get_neighbors(
        self, node: Node, obstacles: Set[Tuple[float, float]], field_bounds: dict
    ) -> List[Node]:
        """Get valid neighboring nodes"""
        neighbors = []
        if DEBUG_PATH_PLANNING:
            print(f"Finding neighbors for node at ({node.x}, {node.y})")
            print(f"Field bounds: {field_bounds}")
            print(f"Number of obstacles: {len(obstacles)}")

        for dx, dy in self.movements:
            new_x = node.x + dx * self.resolution
            new_y = node.y + dy * self.resolution

            # Check if position is valid within field bounds
            if (
                field_bounds["x_min"] <= new_x <= field_bounds["x_max"]
                and field_bounds["y_min"] <= new_y <= field_bounds["y_max"]
            ):
                # Create potential neighbor node position
                neighbor_pos = (round(new_x, 6), round(new_y, 6))

                # Skip if this is an obstacle
                is_obstacle = neighbor_pos in obstacles
                if is_obstacle:
                    if DEBUG_PATH_PLANNING:
                        print(
                            f"  Position ({new_x}, {new_y}) is an obstacle - skipping"
                        )
                    continue

                if DEBUG_PATH_PLANNING:
                    print(f"  Valid neighbor found at ({new_x}, {new_y})")

                # Calculate movement cost
                move_cost = self.resolution * (
                    1.414213562 if dx != 0 and dy != 0 else 1.0
                )

                # Create new node with proper initialization
                new_node = Node(new_x, new_y)
                # Don't set g_cost yet - we'll set it in find_path
                new_node.parent = node

                # Add to neighbors list
                neighbors.append(new_node)
            else:
                if DEBUG_PATH_PLANNING:
                    print(
                        f"  Position ({new_x}, {new_y}) outside field bounds - skipping"
                    )

        if DEBUG_PATH_PLANNING:
            print(f"Total valid neighbors found: {len(neighbors)}")
        return neighbors

    def find_path(
        self,
        start: Tuple[float, float],
        goal: Tuple[float, float],
        field_bounds: dict,
        obstacles: Set[Tuple[float, float]],
    ) -> List[Tuple[float, float]]:
        """Find path using A* algorithm"""
        # Store goal for heuristic calculations
        self.goal_x, self.goal_y = goal

        start_node = Node(start[0], start[1], 0)
        goal_node = Node(goal[0], goal[1])

        # Initialize start node's heuristic
        start_node.h_cost = self.heuristic(start_node, goal_node)

        open_set = []
        open_set_dict = {}  # Dictionary to track nodes in open_set
        closed_set = set()

        heapq.heappush(open_set, start_node)
        open_set_dict[(start_node.x, start_node.y)] = start_node

        while open_set:
            current = heapq.heappop(open_set)
            del open_set_dict[(current.x, current.y)]

            # Debug: Print current node being explored
            if DEBUG_PATH_PLANNING:
                print(
                    f"Exploring Node: ({current.x}, {current.y}), g_cost: {current.g_cost}, h_cost: {current.h_cost}, f_cost: {current.f_cost}"
                )

            # Check if we're close enough to the goal
            if (
                abs(current.x - goal_node.x) < self.resolution
                and abs(current.y - goal_node.y) < self.resolution
            ):
                # Debug: Print the path found
                path = self._reconstruct_path(current)
                if DEBUG_PATH_PLANNING:
                    print(f"Path Found with {len(path)} waypoints")
                return path

            closed_set.add((current.x, current.y))

            # Get neighbors and process them
            neighbors = self.get_neighbors(current, obstacles, field_bounds)
            if DEBUG_PATH_PLANNING:
                print(
                    f"Neighbors of ({current.x}, {current.y}): {[(n.x, n.y) for n in neighbors]}"
                )

            for neighbor in neighbors:
                if (neighbor.x, neighbor.y) in closed_set:
                    continue

                # Calculate tentative g score
                # For diagonal moves, cost is sqrt(2) * resolution
                # For cardinal moves, cost is just resolution
                dx = neighbor.x - current.x
                dy = neighbor.y - current.y
                move_cost = self.resolution * (
                    1.414213562 if dx != 0 and dy != 0 else 1.0
                )
                tentative_g = current.g_cost + move_cost

                neighbor_key = (neighbor.x, neighbor.y)
                if neighbor_key not in open_set_dict:
                    # New node, add to open set
                    neighbor.g_cost = tentative_g
                    neighbor.h_cost = self.heuristic(neighbor, goal_node)
                    neighbor.parent = current
                    heapq.heappush(open_set, neighbor)
                    open_set_dict[neighbor_key] = neighbor
                    if DEBUG_PATH_PLANNING:
                        print(
                            f"Adding Neighbor: ({neighbor.x}, {neighbor.y}), g_cost: {neighbor.g_cost}, h_cost: {neighbor.h_cost}, f_cost: {neighbor.f_cost}"
                        )
                elif tentative_g < open_set_dict[neighbor_key].g_cost:
                    # Better path to existing node, update it
                    existing = open_set_dict[neighbor_key]
                    existing.g_cost = tentative_g
                    existing.parent = current

                    # Need to reheapify after updating
                    heapq.heapify(open_set)
                    if DEBUG_PATH_PLANNING:
                        print(
                            f"Updating Neighbor: ({existing.x}, {existing.y}), g_cost: {existing.g_cost}, h_cost: {existing.h_cost}, f_cost: {existing.f_cost}"
                        )

        # Debug: Print if no path is found
        if DEBUG_PATH_PLANNING:
            print("No Path Found")
        return []  # No path found

    def _reconstruct_path(self, node: Node) -> List[Tuple[float, float]]:
        """Reconstruct path from goal to start"""
        path = []
        current = node

        while current:
            path.append((current.x, current.y))
            current = current.parent

        return path[::-1]
