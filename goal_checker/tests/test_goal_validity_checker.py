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
import rclpy
from goal_checker.goal_validity_checker import GoalCheckerService

from nav_msgs.msg import MapMetaData


class TestGoalValidityChecker(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.goal_checker = GoalCheckerService()

    def tearDown(self):
        rclpy.shutdown()

    def test_distance_between_2_points(self):

        result_1 = self.goal_checker.distance_between_2_points(0, 0, 0, 0)
        result_2 = self.goal_checker.distance_between_2_points(1, 0, 3, 0)
        result_3 = self.goal_checker.distance_between_2_points(0, 5, 0, 0)
        result_4 = self.goal_checker.distance_between_2_points(-115, 5, -115, 5)
        self.assertEqual(result_1, 0)
        self.assertEqual(result_2, 2)
        self.assertEqual(result_3, 5)
        self.assertEqual(result_4, 0)

    def test_get_cell_list(self):
        self.goal_checker.map_info = MapMetaData(width=101)
        list = self.goal_checker.get_cell_list()
        self.assertListEqual(list, [1, -1, 101, -101, 100, 102, -102, -100])
        list_2 = self.goal_checker.get_cell_list(weight=2, weight_diag=2)
        self.assertListEqual(list_2, [2, -2, 202, -202, 200, 204, -204, -200])

    def test_reset(self):
        self.goal_checker.weight = 10
        self.goal_checker.weight_diag = -5
        self.goal_checker.map_info = MapMetaData(width=100, height=155)
        self.goal_checker.cell_list = [0, 1, 2, 3, 4, 5, 6, 7]
        self.goal_checker.in_defined_range = False
        self.goal_checker.new_goal_found = True
        self.goal_checker.reset()
        self.assertEqual(self.goal_checker.weight, 1)
        self.assertEqual(self.goal_checker.weight_diag, 1)
        self.assertEqual(self.goal_checker.map_info.width, 100)
        self.assertEqual(self.goal_checker.map_info.height, 155)
        self.assertListEqual(
            self.goal_checker.cell_list,
            [
                self.goal_checker.weight,
                -self.goal_checker.weight,
                self.goal_checker.map_info.width * self.goal_checker.weight,
                -self.goal_checker.map_info.width * self.goal_checker.weight,
                (self.goal_checker.map_info.width - 1) * self.goal_checker.weight_diag,
                (self.goal_checker.map_info.width + 1) * self.goal_checker.weight_diag,
                -(self.goal_checker.map_info.width + 1) * self.goal_checker.weight_diag,
                -(self.goal_checker.map_info.width - 1) * self.goal_checker.weight_diag,
            ],
        )
        self.assertEqual(self.goal_checker.in_defined_range, True)
        self.assertEqual(self.goal_checker.new_goal_found, False)

    def test_get_occupancy(self):
        list = [1, 0, 5, 8, 28, 155, 98, 4, 0, 0, 65]
        self.goal_checker.costmap = list

        self.assertEqual(
            self.goal_checker.get_occupancy(self.goal_checker.costmap, 0), 1
        )
        self.assertEqual(
            self.goal_checker.get_occupancy(self.goal_checker.costmap, 2), 5
        )
        self.assertEqual(
            self.goal_checker.get_occupancy(self.goal_checker.costmap, 1), 0
        )
        self.assertEqual(
            self.goal_checker.get_occupancy(self.goal_checker.costmap, 3), 8
        )

    def test_expand_research(self):
        self.goal_checker.weight = 4
        self.goal_checker.weight_diag = 4
        self.goal_checker.map_info = MapMetaData(width=100)
        self.goal_checker.cell_list = [0, 1, 2, 3, 4, 5, 6, 7]
        self.goal_checker.expand_research()
        self.assertEqual(self.goal_checker.weight, 5)
        self.assertEqual(self.goal_checker.weight_diag, 4 + 1)
        self.assertListEqual(
            self.goal_checker.cell_list,
            [
                self.goal_checker.weight,
                -self.goal_checker.weight,
                self.goal_checker.map_info.width * self.goal_checker.weight,
                -self.goal_checker.map_info.width * self.goal_checker.weight,
                (self.goal_checker.map_info.width - 1) * self.goal_checker.weight_diag,
                (self.goal_checker.map_info.width + 1) * self.goal_checker.weight_diag,
                -(self.goal_checker.map_info.width + 1) * self.goal_checker.weight_diag,
                -(self.goal_checker.map_info.width - 1) * self.goal_checker.weight_diag,
            ],
        )

    def test_get_index(self):
        x_map = 10
        y_map = 30
        self.goal_checker.map_info = MapMetaData(width=100)
        index = self.goal_checker.get_index(x_map, y_map)
        self.assertEqual(index, 3010)

    def test_get_max_weights(self):
        self.goal_checker.map_info = MapMetaData(resolution=0.05)
        self.goal_checker.max_range_new_goal = 2.0
        max_weight, max_weight_diag = self.goal_checker.get_max_weights()
        self.assertEqual(max_weight, 41)
        self.assertEqual(max_weight_diag, 28)

        self.goal_checker.map_info = MapMetaData(resolution=0.0)
        self.goal_checker.max_range_new_goal = 2.0
        with self.assertRaises(ZeroDivisionError):
            self.goal_checker.get_max_weights()

    def test_get_xy_map(self):
        self.goal_checker.map_info = MapMetaData(resolution=0.05)
        self.goal_checker.map_info.origin.position.x = 2.1
        self.goal_checker.map_info.origin.position.y = 3.1
        x_map, y_map = self.goal_checker.get_xy_map(4, 6)

    def test_new_goal_pose_x(self):
        self.goal_checker.new_goal_found = False
        self.goal_checker.map_info = MapMetaData(width=100, resolution=0.05)
        self.goal_checker.map_info.origin.position.x = 1.0
        self.goal_checker.map_info.origin.position.y = 0.0
        new_x, new_y = self.goal_checker.get_new_goal_pose_x(1, 0, 3, 0)
        self.assertEqual(new_y, 3)
        self.assertEqual(new_x, 1.05)
        self.assertEqual(self.goal_checker.new_goal_found, True)
        modulo = 247 % 124
        self.assertEqual(modulo, 123)


if __name__ == "__main__":
    unittest.main()
