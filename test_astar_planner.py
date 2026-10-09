import unittest

from gps_denied_navigation.astar_planner import astar_grid


class AStarPlannerTests(unittest.TestCase):
    def test_open_grid_returns_start_and_goal(self):
        path = astar_grid(5, 5, (0, 0), (4, 4), blocked=set())
        self.assertIsNotNone(path)
        self.assertEqual(path[0], (0, 0))
        self.assertEqual(path[-1], (4, 4))

    def test_blocked_goal_has_no_path(self):
        self.assertIsNone(astar_grid(5, 5, (0, 0), (4, 4), blocked={(4, 4)}))

    def test_obstacle_is_not_in_path(self):
        blocked = {(2, 2), (2, 1), (2, 3)}
        path = astar_grid(5, 5, (0, 2), (4, 2), blocked=blocked)
        self.assertIsNotNone(path)
        self.assertTrue(all(cell not in blocked for cell in path))

    def test_diagonal_corner_cutting_is_prevented(self):
        self.assertIsNone(astar_grid(2, 2, (0, 0), (1, 1), blocked={(1, 0), (0, 1)}))

    def test_start_equals_goal(self):
        self.assertEqual(astar_grid(3, 3, (1, 1), (1, 1), blocked=set()), [(1, 1)])

    def test_out_of_bounds_returns_none(self):
        self.assertIsNone(astar_grid(3, 3, (-1, 0), (2, 2), blocked=set()))


if __name__ == '__main__':
    unittest.main()
