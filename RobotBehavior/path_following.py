from .robot_state_machine import DEBUG_ROBOT_BEHAVIOR


class PathFollower:
    def __init__(self, lookahead_distance=0.2):
        self.lookahead_distance = lookahead_distance
        if DEBUG_ROBOT_BEHAVIOR:
            print(
                f"PathFollower initialized with lookahead distance: {lookahead_distance}"
            )

    def get_target_point(
        self, current_pos: Tuple[float, float], path: List[Tuple[float, float]]
    ) -> Tuple[float, float]:
        """Get target point along path based on lookahead distance"""
        if not path:
            if DEBUG_ROBOT_BEHAVIOR:
                print("WARNING: Empty path provided to PathFollower")
            return current_pos

        # If we're close to the end, return the end point
        end_distance = self._distance(current_pos, path[-1])
        if end_distance < self.lookahead_distance:
            if DEBUG_ROBOT_BEHAVIOR:
                print(
                    f"Close to end point ({end_distance:.2f}m), targeting end directly"
                )
            return path[-1]

        # Find the point along the path that's approximately lookahead_distance away
        for i in range(len(path)):
            if self._distance(current_pos, path[i]) >= self.lookahead_distance:
                if DEBUG_ROBOT_BEHAVIOR:
                    print(f"Found lookahead point at index {i} in path")
                return path[i]

        # Default to last point
        if DEBUG_ROBOT_BEHAVIOR:
            print("No suitable point found, using last path point")
        return path[-1]

    def _distance(self, point1, point2):
        """Calculate Euclidean distance between two points"""
        return ((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2) ** 0.5
