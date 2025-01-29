import matplotlib.pyplot as plt
from astar import AStar, Node
from typing import Set, Tuple, List
from utils import inflate_obstacles


def visualize_astar(
    start: Tuple[float, float],
    goal: Tuple[float, float],
    field_bounds: dict,
    obstacles: Set[Tuple[float, float]],
    inflated_obstacles: Set[Tuple[float, float]],
    resolution: float = 0.05,
):
    # Initialize A* algorithm
    astar = AStar(resolution=resolution)

    # Find the path
    path = astar.find_path(start, goal, field_bounds, inflated_obstacles)

    fig, ax = plt.subplots(figsize=(10, 10))

    # Plot field boundaries
    ax.plot(
        [field_bounds["x_min"], field_bounds["x_max"]],
        [field_bounds["y_min"], field_bounds["y_min"]],
        "k-",
        linewidth=2,
    )
    ax.plot(
        [field_bounds["x_min"], field_bounds["x_max"]],
        [field_bounds["y_max"], field_bounds["y_max"]],
        "k-",
        linewidth=2,
    )
    ax.plot(
        [field_bounds["x_min"], field_bounds["x_min"]],
        [field_bounds["y_min"], field_bounds["y_max"]],
        "k-",
        linewidth=2,
    )
    ax.plot(
        [field_bounds["x_max"], field_bounds["x_max"]],
        [field_bounds["y_min"], field_bounds["y_max"]],
        "k-",
        linewidth=2,
    )

    # Plot original obstacles with larger, more visible markers
    for obs in obstacles:
        circle = plt.Circle(obs, 0.05, color="red", alpha=0.5)
        ax.add_artist(circle)

    # Plot inflated obstacles with transparency
    for obs in inflated_obstacles:
        circle = plt.Circle(obs, resolution / 2, color="pink", alpha=0.3)
        ax.add_artist(circle)

    # Plot start and goal with clear markers
    ax.plot(start[0], start[1], "go", markersize=15, label="Start")
    ax.plot(goal[0], goal[1], "ro", markersize=15, label="Goal")

    # Plot path if found
    if path:
        path_x, path_y = zip(*path)
        ax.plot(path_x, path_y, "b-", linewidth=2, label="Path")

        # Plot nodes along path
        ax.plot(path_x, path_y, "b.", markersize=5)

    # Set grid and labels
    ax.grid(True, linestyle="--", alpha=0.6)
    ax.set_xlabel("X (meters)")
    ax.set_ylabel("Y (meters)")

    # Add some margin to the plot
    margin = 0.2
    ax.set_xlim(field_bounds["x_min"] - margin, field_bounds["x_max"] + margin)
    ax.set_ylim(field_bounds["y_min"] - margin, field_bounds["y_max"] + margin)

    ax.legend(loc="upper left")
    ax.set_aspect("equal")

    plt.title("A* Pathfinding with Obstacle Avoidance")
    plt.show()


if __name__ == "__main__":
    # Define field bounds (using SSL-EL field dimensions)
    field_bounds = {"x_min": -2.25, "x_max": 2.25, "y_min": -1.5, "y_max": 1.5}

    # Define some test obstacles
    obstacles = {(0.0, 0.0), (0.5, 0.5), (-0.5, -0.5), (0.16, .5),(1.0, 0.0), (-1.0, 0.0)}

    # Inflate obstacles
    inflation_radius = 0.15  # Robot radius + safety margin
    inflated_obstacles = inflate_obstacles(obstacles, inflation_radius, resolution=0.05)

    # Test different start and goal positions
    start = (-1, -1.0)
    goal = (1.5, 1.0)

    # Visualize A* algorithm
    visualize_astar(
        start=start,
        goal=goal,
        field_bounds=field_bounds,
        obstacles=obstacles,
        inflated_obstacles=inflated_obstacles,
        resolution=0.05,
    )
