# test_path_follower.py
import unittest
from RobotBehavior.robot_state_machine import PathFollower


class TestPathFollower(unittest.TestCase):
    def setUp(self):
        self.follower = PathFollower(lookahead_distance=0.3)

    def test_get_target_point_empty_path(self):
        """Test behavior with empty path"""
        current_pos = (1.0, 1.0)
        path = []
        result = self.follower.get_target_point(current_pos, path)
        self.assertEqual(result, current_pos)

    def test_get_target_point_short_path(self):
        """Test behavior with short path"""
        current_pos = (0.0, 0.0)
        path = [(1.0, 0.0), (2.0, 0.0)]
        result = self.follower.get_target_point(current_pos, path)
        self.assertEqual(result, (1.0, 0.0))

    def test_distance_calculation(self):
        """Test distance calculation"""
        point1 = (0.0, 0.0)
        point2 = (3.0, 4.0)
        distance = self.follower._distance(point1, point2)
        self.assertEqual(distance, 5.0)


if __name__ == "__main__":
    unittest.main()
