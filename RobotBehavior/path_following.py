# In a new file robot_behavior/path_following.py
class PathFollower:
    def __init__(self, lookahead_distance=0.2):
        self.lookahead_distance = lookahead_distance

    def get_target_point(self, current_pos, path):
        """Get target point along path based on lookahead distance"""
        if not path:
            return None

        # If we're close to the end, return the end point
        if self._distance(current_pos, path[-1]) < self.lookahead_distance:
            return path[-1]

        # Find the point along the path that's approximately lookahead_distance away
        for i in range(len(path) - 1):
            if self._distance(current_pos, path[i]) >= self.lookahead_distance:
                return path[i]

        # Default to last point
        return path[-1]

    def _distance(self, point1, point2):
        """Calculate Euclidean distance between two points"""
        return ((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2) ** 0.5
