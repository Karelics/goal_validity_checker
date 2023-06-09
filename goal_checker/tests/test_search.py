#!/usr/bin/env python3

#    Goal Validity Checker - ROS 2 Node checking valid goal poses during navigation.
#    Copyright (C) 2023  Karelics Oy
#
#    This program is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with this program. If not, see <https://www.gnu.org/licenses/>.

import unittest

# import rclpy
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Point

from goal_checker.search import Search, State

import rclpy


class TestGoalValidityChecker(unittest.TestCase):
    def test_intially_free_valid(self):
        """Initially free goal point surrounded by at least min_range of free space"""
        search = Search(self.get_occupancy_grid(), 0.5, 65, False, 0.2)
        goal_point = Point(x=0.6, y=0.5)
        result = search.check_for_goal_pose(goal_point)
        self.assertEqual(result.state, State.VALID)
        self.assertEqual(result.goal, goal_point)

    def test_intially_free_not_valid(self):
        """Initially free goal point which is not surrounded by min_range of free space"""
        search = Search(self.get_occupancy_grid(), 0.5, 65, False, 0.2)
        goal_point = Point(x=0.4, y=0.1)
        result = search.check_for_goal_pose(goal_point)
        self.assertEqual(result.state, State.NEW_GOAL_FOUND)
        self.assertAlmostEqual(result.goal.x, 0.4)
        self.assertAlmostEqual(result.goal.y, 0.3)

    def test_floating_point_error_case(self):
        """Test for min_range case where floating point error can happen.
        min_range 0.3 and resolution 0.1 for example
        Should be 3 cells from the edge of costmap
        """
        search = Search(self.get_occupancy_grid(), 0.5, 65, False, 0.3)
        min_weight = search._get_min_range_weights()
        self.assertEqual(min_weight, 3)

    def test_unknown(self):
        """Case which checks if the set_unknown_area_search parameter works correctly by setting it to False"""
        search = Search(self.get_occupancy_grid(), 0.5, 65, False, 0.2)
        goal_point = Point(x=0.3, y=0.0)
        result = search.check_for_goal_pose(goal_point)
        self.assertEqual(result.state, State.UNKNOWN)

    def test_set_unknown_area_search(self):
        """Case which checks if the set_unknown_area_search parameter works correctly by setting it to True"""
        search = Search(self.get_occupancy_grid(), 0.8, 65, True, 0.2)
        goal_point = Point(x=0.3, y=0.0)
        result = search.check_for_goal_pose(goal_point)
        self.assertEqual(result.state, State.NEW_GOAL_FOUND)
        self.assertAlmostEqual(result.goal.x, 0.6)
        self.assertAlmostEqual(result.goal.y, 0.3)

    def test_out_of_range(self):
        """Test case where result should be out of range"""
        search = Search(self.get_occupancy_grid(), 0.4, 0.65, False, 0.3)
        goal_point = Point(x=0.1, y=0.3)
        result = search.check_for_goal_pose(goal_point)
        self.assertEqual(result.state, State.OUT_OF_RANGE)
        self.assertAlmostEqual(result.goal.x, 0.0)
        self.assertAlmostEqual(result.goal.y, 0.0)

    def test_simple_occupied_case(self):
        """Simple case where intial goal on occupied but new goal found on x axis"""
        search = Search(self.get_occupancy_grid(), 0.5, 0.65, False, 0.0)
        goal_point = Point(x=0.1, y=0.3)
        result = search.check_for_goal_pose(goal_point)
        self.assertEqual(result.state, State.NEW_GOAL_FOUND)
        self.assertAlmostEqual(result.goal.x, 0.2)
        self.assertAlmostEqual(result.goal.y, 0.3)

    def test_simple_occupied_min_range_case(self):
        """Simple case where we test the min_range with an intial goal on occupied but new goal found on x axis"""
        search = Search(self.get_occupancy_grid(), 0.5, 0.65, False, 0.2)
        goal_point = Point(x=0.1, y=0.3)
        result = search.check_for_goal_pose(goal_point)
        self.assertEqual(result.state, State.NEW_GOAL_FOUND)
        self.assertAlmostEqual(result.goal.x, 0.4)
        self.assertAlmostEqual(result.goal.y, 0.3)

    # Tests for boundary condition searches

    def test_boundary_condition_left(self):
        """Case which only takes care of the boundary condition on left side"""
        search = Search(self.get_occupancy_grid(), 0.5, 65, False, 0.2)
        goal_point = Point(x=0.0, y=0.4)
        result = search.check_for_goal_pose(goal_point)
        self.assertEqual(result.state, State.NEW_GOAL_FOUND)
        self.assertAlmostEqual(result.goal.x, 0.4)
        self.assertAlmostEqual(result.goal.y, 0.4)

    def test_boundary_condition_left_improved(self):
        """Case which takes care of the boundary condition on left side"""
        search = Search(self.get_occupancy_grid_v3(), 0.5, 65, False, 0.1)
        goal_point = Point(x=0.2, y=0.4)
        result = search.check_for_goal_pose(goal_point)
        self.assertEqual(result.state, State.NEW_GOAL_FOUND)
        self.assertAlmostEqual(result.goal.x, 0.2)
        self.assertAlmostEqual(result.goal.y, 0.6)

    def test_boundary_condition_right(self):
        """
        DOES NOT PASS
        Case which only takes care of the boundary condition on right side"""
        search = Search(self.get_occupancy_grid_v2(), 0.5, 65, False, 0.0)
        goal_point = Point(x=0.9, y=0.5)
        result = search.check_for_goal_pose(goal_point)
        self.assertEqual(result.state, State.NEW_GOAL_FOUND)
        self.assertAlmostEqual(result.goal.x, 0.8)
        self.assertAlmostEqual(result.goal.y, 0.5)

    def test_boundary_condition_top(self):
        """Case which only takes care of the boundary condition on the top"""
        search = Search(self.get_occupancy_grid_v2(), 0.5, 65, False, 0.2)
        goal_point = Point(x=0.3, y=0.0)
        result = search.check_for_goal_pose(goal_point)
        self.assertEqual(result.state, State.NEW_GOAL_FOUND)
        self.assertAlmostEqual(result.goal.x, 0.3)
        self.assertAlmostEqual(result.goal.y, 0.3)

    def test_boundary_condition_bottom(self):
        """Case which only takes care of the boundary condition on the bottom"""
        search = Search(self.get_occupancy_grid_v2(), 0.5, 65, False, 0.2)
        goal_point = Point(x=0.3, y=0.9)
        result = search.check_for_goal_pose(goal_point)
        self.assertEqual(result.state, State.NEW_GOAL_FOUND)
        self.assertAlmostEqual(result.goal.x, 0.3)
        self.assertAlmostEqual(result.goal.y, 0.5)

    def test_boundary_condition_top_left_corner(self):
        """Case which only takes care of the top left boundary condition corner"""
        search = Search(self.get_occupancy_grid_v2(), 0.5, 65, False, 0.0)
        goal_point = Point(x=0.0, y=0.0)
        result = search.check_for_goal_pose(goal_point)
        self.assertEqual(result.state, State.NEW_GOAL_FOUND)
        self.assertAlmostEqual(result.goal.x, 0.0)
        self.assertAlmostEqual(result.goal.y, 0.1)

    def test_boundary_condition_bottom_left_corner(self):
        """Case which only takes care of the bottom left boundary condition corner"""
        search = Search(self.get_occupancy_grid_v2(), 0.5, 65, False, 0.0)
        goal_point = Point(x=0.0, y=0.9)
        result = search.check_for_goal_pose(goal_point)
        self.assertEqual(result.state, State.NEW_GOAL_FOUND)
        self.assertAlmostEqual(result.goal.x, 0.0)
        self.assertAlmostEqual(result.goal.y, 0.7)

    def test_boundary_condition_top_right_corner(self):
        """Case which only takes care of the top right boundary condition corner"""
        search = Search(self.get_occupancy_grid_v2(), 0.5, 65, False, 0.0)
        goal_point = Point(x=0.9, y=0.0)
        result = search.check_for_goal_pose(goal_point)
        self.assertEqual(result.state, State.NEW_GOAL_FOUND)
        self.assertAlmostEqual(result.goal.x, 0.6)
        self.assertAlmostEqual(result.goal.y, 0.3)

    def test_boundary_condition_bottom_right_corner(self):
        """Case which only takes care of the bottom right boundary condition corner"""
        search = Search(self.get_occupancy_grid_v2(), 0.5, 65, False, 0.0)
        goal_point = Point(x=0.9, y=0.9)
        result = search.check_for_goal_pose(goal_point)
        self.assertEqual(result.state, State.NEW_GOAL_FOUND)
        self.assertAlmostEqual(result.goal.x, 0.7)
        self.assertAlmostEqual(result.goal.y, 0.7)

    # Tests for min_range_search boundary conditions

    def test_boundary_condition_min_range_search_left(self):
        """Case which only takes care of the boundary condition of the min_range_search on left side"""
        search = Search(self.get_occupancy_grid_v2(), 0.5, 65, False, 0.2)
        goal_point = Point(x=0.1, y=0.0)
        result = search.check_for_goal_pose(goal_point)
        self.assertEqual(result.state, State.NEW_GOAL_FOUND)
        self.assertAlmostEqual(result.goal.x, 0.4)
        self.assertAlmostEqual(result.goal.y, 0.3)

    def test_boundary_condition_min_range_search_right(self):
        """Case which only takes care of the boundary condition of the min_range_search on right side"""
        search = Search(self.get_occupancy_grid(), 0.8, 0.65, False, 0.2)
        goal_point = Point(x=0.8, y=0.1)
        result = search.check_for_goal_pose(goal_point)
        self.assertEqual(result.state, State.NEW_GOAL_FOUND)
        self.assertAlmostEqual(result.goal.x, 0.6)
        self.assertAlmostEqual(result.goal.y, 0.3)

    def test_boundary_condition_min_range_search_top(self):
        """Case which only takes care of the boundary condition of the min_range_search on the top"""
        search = Search(self.get_occupancy_grid_v2(), 0.5, 65, False, 0.2)
        goal_point = Point(x=0.4, y=0.1)
        result = search.check_for_goal_pose(goal_point)
        self.assertEqual(result.state, State.NEW_GOAL_FOUND)
        self.assertAlmostEqual(result.goal.x, 0.4)
        self.assertAlmostEqual(result.goal.y, 0.3)

    def test_boundary_condition_min_range_search_bottom(self):
        """Case which only takes care of the boundary condition of the min_range_search on the bottom"""
        search = Search(self.get_occupancy_grid_v2(), 0.5, 65, False, 0.2)
        goal_point = Point(x=0.4, y=0.8)
        result = search.check_for_goal_pose(goal_point)
        self.assertEqual(result.state, State.NEW_GOAL_FOUND)
        self.assertAlmostEqual(result.goal.x, 0.4)
        self.assertAlmostEqual(result.goal.y, 0.6)

    @staticmethod
    def get_occupancy_grid() -> OccupancyGrid:
        """Creates an Occupancygrid with possible goal point at (0.3, 0.3) if min_range is 0.2"""
        # fmt: off
        grid = [
            100, 100, 100, -1, 100, 100, 100, 100, 100, 100,
            100, 100,  0 ,  0,   0,   0,   0,   0,   0,   0,
            100, 100,  0 ,  0,   0,   0,   0,   0,   0,   0,
            100, 100,  0 ,  0,   0,   0,   0,   0,   0,   0,
            100, 100,  0 ,  0,   0,   0,   0,   0,   0,   0,
            100, 100,  0 ,  0,   0,   0,   0,   0,   0,   0,
            100, 100,  0 ,  0,   0,   0,   0,   0,   0,   0,
            100, 100,  0 ,  0,   0,   0,   0,   0,   0,   0,
            100, 100,  0 ,  0,   0,   0,   0,   0,   0,   0,
            100, 100, 100, 100, 100, 100, 100, 100, 100, 100,

        ]

        # fmt: on

        meta_data = MapMetaData(resolution=0.1, width=10, height=10)
        return OccupancyGrid(info=meta_data, data=grid)

    @staticmethod
    def get_occupancy_grid_v2() -> OccupancyGrid:
        """Creates an Occupancygrid with possible goal point at (0.3, 0.3) if min_range is 0.2"""
        # fmt: off
        
        grid = [
            100, 100, 100, 100, 100, 100, 100, 100,  100, 100,
             0,   0,   0 ,  0,   0,   0,  0,   100,  100, 100,
             0,   0,   0 ,  0,   0,   0,  0,   100,  100, 100,
             0,   0,   0 ,  0,   0,   0,  0,   100,  100, 100,
             0,   0,   0 ,  0,   0,   0,  0,   100,  100, 100,
             0,   0,   0 ,  0,   0,   0,  0,   100,   0,  100,
             0,   0,   0 ,  0,   0,   0,  0,   100,  100, 100,
             0,   0,   0 ,  0,   0,   0,  0,    0,   100, 100,
            100, 100,  0 ,  0,   0,   0,  0,   100,  100, 100,
            100, 100, 100, 100, 100, 100, 100, 100,  100, 100,
        ]
        # fmt: on

        meta_data = MapMetaData(resolution=0.1, width=10, height=10)
        return OccupancyGrid(info=meta_data, data=grid)

    @staticmethod
    def get_occupancy_grid_v3() -> OccupancyGrid:
        """Creates an Occupancygrid with possible goal point at (0.3, 0.3) if min_range is 0.2"""
        # fmt: off
        
        grid = [
            100, 100, 100, -1,  100, 100, 100, 100, 100, 100,
            100, 100,  0 ,  0,   0,  100,  0,   0,   0,   0,
            100, 100, 100, 100, 100, 100,  0,   0,   0,   0,
            100, 100, 100, 100, 100, 100,  0,   0,   0,   0,
            100, 100, 100, 100, 100, 100,  0,   0,   0,   0,
            100, 0  ,  0 ,  0,   0,  100,  0,   0,   0,   0,
            100, 0  ,  0 ,  0,   0,  100,  0,   0,   0,   0,
            100, 0  ,  0 ,  0,   0,  100,  0,   0,   0,   0,
            100, 100, 100, 100, 100, 100,  0,   0,   0,   0,
            100, 100, 100, 100, 100, 100, 100, 100, 100, 100,

        ]
        # fmt: on

        meta_data = MapMetaData(resolution=0.1, width=10, height=10)
        return OccupancyGrid(info=meta_data, data=grid)


if __name__ == "__main__":
    unittest.main()
