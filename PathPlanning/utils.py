import numpy as np
from typing import Set, Tuple, List


def inflate_obstacles(
    obstacles: Set[Tuple[float, float]],
    inflation_radius: float,
    resolution: float,
    field_bounds: dict = None,
) -> Set[Tuple[float, float]]:
    """
    Inflate obstacles by a given radius using efficient NumPy operations.

    Args:
        obstacles: Set of tuples (x, y) representing obstacle positions.
        inflation_radius: Radius to inflate each obstacle.
        resolution: Grid resolution.
        field_bounds: Optional dictionary with field boundaries {x_min, x_max, y_min, y_max}.
                     If provided, inflated obstacles will be constrained to field bounds.

    Returns:
        Set of tuples representing inflated obstacles.
    """
    if not obstacles:
        return set()

    # Convert obstacles to numpy array for efficient operations
    obstacles_array = np.array(list(obstacles))

    # Create grid of points for inflation
    radius_steps = int(np.ceil(inflation_radius / resolution))
    x_offsets = np.arange(-radius_steps, radius_steps + 1) * resolution
    y_offsets = np.arange(-radius_steps, radius_steps + 1) * resolution
    X_offsets, Y_offsets = np.meshgrid(x_offsets, y_offsets)

    # Create mask for points within inflation radius
    mask = X_offsets**2 + Y_offsets**2 <= inflation_radius**2
    valid_offsets = np.column_stack((X_offsets[mask], Y_offsets[mask]))

    # Initialize set for inflated obstacles
    inflated_obstacles = set()

    # Apply offsets to each obstacle
    for obs in obstacles_array:
        inflated_points = obs + valid_offsets

        # Apply field bounds constraint if provided
        if field_bounds is not None:
            mask = (
                (inflated_points[:, 0] >= field_bounds["x_min"])
                & (inflated_points[:, 0] <= field_bounds["x_max"])
                & (inflated_points[:, 1] >= field_bounds["y_min"])
                & (inflated_points[:, 1] <= field_bounds["y_max"])
            )
            inflated_points = inflated_points[mask]

        # Round coordinates to avoid floating-point issues
        inflated_points = np.round(inflated_points, decimals=3)

        # Add points to set
        inflated_obstacles.update(map(tuple, inflated_points))

    return inflated_obstacles
