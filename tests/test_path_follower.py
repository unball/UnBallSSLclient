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

    def test_get_target_point_longer_path(self):
        """Test behavior with a longer path and lookahead"""
        current_pos = (0.0, 0.0)
        path = [(0.1, 0.0), (0.2, 0.0), (0.4, 0.0), (0.7, 0.0)]
        result = self.follower.get_target_point(current_pos, path)
        # The target should be the first point farther than lookahead_distance (0.3)
        self.assertEqual(result, (0.4, 0.0))

    def test_distance_calculation(self):
        """Test distance calculation"""
        point1 = (0.0, 0.0)
        point2 = (3.0, 4.0)
        distance = self.follower._distance(point1, point2)
        self.assertAlmostEqual(distance, 5.0)

    def test_get_target_point_exact_lookahead(self):
        """Test when a point is exactly at lookahead distance"""
        current_pos = (0.0, 0.0)
        path = [(0.3, 0.0), (0.6, 0.0)]
        result = self.follower.get_target_point(current_pos, path)
        self.assertEqual(result, (0.3, 0.0))


if __name__ == "__main__":
    unittest.main()
