# pathplanning/astar.py
from dataclasses import dataclass
import heapq
import numpy as np
from typing import List, Tuple, Set


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

    def heuristic(self, node: Node, goal: Node) -> float:
        """Manhattan distance heuristic"""
        return abs(node.x - goal.x) + abs(node.y - goal.y)

    def get_neighbors(
        self, node: Node, obstacles: Set[Tuple[float, float]], field_bounds: dict
    ) -> List[Node]:
        """Get valid neighboring nodes"""
        neighbors = []
        for dx, dy in self.movements:
            new_x = node.x + dx * self.resolution
            new_y = node.y + dy * self.resolution

            # Create potential neighbor node position
            neighbor_pos = (
                round(new_x, 6),
                round(new_y, 6),
            )  # Round to avoid floating point issues

            # Check if position is valid:
            # 1. Within field bounds
            # 2. Not in obstacles
            # 3. Has enough clearance from obstacles
            if (
                field_bounds["x_min"] <= new_x <= field_bounds["x_max"]
                and field_bounds["y_min"] <= new_y <= field_bounds["y_max"]
                and neighbor_pos not in obstacles
            ):
                # Calculate movement cost: sqrt(2) for diagonals, 1 for cardinals
                cost = self.resolution * (1.414213562 if dx != 0 and dy != 0 else 1.0)

                # Create new node with updated cost
                new_node = Node(new_x, new_y)
                new_node.g_cost = node.g_cost + cost

                # Check for minimum clearance from obstacles
                min_clearance = self.resolution * 2  # Adjust this value as needed
                is_safe = True

                for obs_x, obs_y in obstacles:
                    dist = ((new_x - obs_x) ** 2 + (new_y - obs_y) ** 2) ** 0.5
                    if dist < min_clearance:
                        is_safe = False
                        break

                if is_safe:
                    neighbors.append(new_node)

        return neighbors

    def find_path(
        self,
        start: Tuple[float, float],
        goal: Tuple[float, float],
        field_bounds: dict,
        obstacles: Set[Tuple[float, float]],
    ) -> List[Tuple[float, float]]:
        """Find path using A* algorithm"""
        start_node = Node(start[0], start[1], 0)
        goal_node = Node(goal[0], goal[1])

        open_set = []
        open_set_dict = {}  # Dictionary to track nodes in open_set
        closed_set = set()

        heapq.heappush(open_set, start_node)
        open_set_dict[(start_node.x, start_node.y)] = start_node

        while open_set:
            current = heapq.heappop(open_set)
            del open_set_dict[(current.x, current.y)]

            # Debug: Print current node being explored
            print(
                f"Exploring Node: ({current.x}, {current.y}), g_cost: {current.g_cost}, h_cost: {current.h_cost}, f_cost: {current.f_cost}"
            )

            if (
                abs(current.x - goal_node.x) < self.resolution
                and abs(current.y - goal_node.y) < self.resolution
            ):
                # Debug: Print the path found
                path = self._reconstruct_path(current)
                print("Path Found:", path)
                return path

            closed_set.add((current.x, current.y))

            # Debug: Print closed set
            print("Closed Set:", closed_set)

            # Get neighbors and print them
            neighbors = self.get_neighbors(current, obstacles, field_bounds)
            print(
                f"Neighbors of ({current.x}, {current.y}): {[(n.x, n.y) for n in neighbors]}"
            )

            for neighbor in neighbors:
                if (neighbor.x, neighbor.y) in closed_set:
                    continue

                tentative_g = current.g_cost + self.resolution

                if (neighbor.x, neighbor.y) not in open_set_dict:
                    neighbor.g_cost = tentative_g
                    neighbor.h_cost = self.heuristic(neighbor, goal_node)
                    neighbor.parent = current
                    heapq.heappush(open_set, neighbor)
                    open_set_dict[(neighbor.x, neighbor.y)] = neighbor

                    # Debug: Print neighbor details
                    print(
                        f"Adding Neighbor: ({neighbor.x}, {neighbor.y}), g_cost: {neighbor.g_cost}, h_cost: {neighbor.h_cost}, f_cost: {neighbor.f_cost}"
                    )
                elif tentative_g < neighbor.g_cost:
                    neighbor.g_cost = tentative_g
                    neighbor.parent = current
                    heapq.heapify(open_set)

                    # Debug: Print updated neighbor details
                    print(
                        f"Updating Neighbor: ({neighbor.x}, {neighbor.y}), g_cost: {neighbor.g_cost}, h_cost: {neighbor.h_cost}, f_cost: {neighbor.f_cost}"
                    )

        # Debug: Print if no path is found
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
