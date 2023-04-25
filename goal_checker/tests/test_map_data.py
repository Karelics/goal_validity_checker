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

#!/usr/bin/env python3

import unittest
from nav_msgs.msg import OccupancyGrid, MapMetaData

from goal_checker.map_data import MapData
from geometry_msgs.msg import Point


class TestGoalValidityChecker(unittest.TestCase):
    @staticmethod
    def get_occupancy_grid() -> OccupancyGrid:
        """Creates an Occupancygrid"""
        # fmt: off
        grid = [
             0,  1,  2,  3,  4,  5, 6,
             7,  8,  9, 10, 11, 12, 13,
            14, 15, 16, 17, 18, 19, 20,
            21, 22, 23, 24, 25, 26, 27,
            28, 29, 30, 31, 32, 33, 34,
            35, 36, 37, 38, 39, 40, 41,
            42, 43, 44, 45, 46, 47, 48
        ]
        # fmt: on

        meta_data = MapMetaData(resolution=0.1, width=7, height=7)
        return OccupancyGrid(info=meta_data, data=grid)

    @staticmethod
    def get_occupancy_grid_v2() -> OccupancyGrid:
        """Creates an Occupancygrid"""
        # fmt: off
 
        grid = [
            100, 67,  47,  67,  67,  0,  0,
            100, 67,  47,  67,  67,  0,  0,
            67,  29,  47,  47,  67,  67, 67,
            47,  47,  47,  47,  67,  67, 67,
            0,   0,   0,   47,  47,  67, 67,
            -1,   0,   0,   47,  47,  67, 47,
            0,   0,   0,   47,  47,  47,  0
                ]
        # fmt: on

        meta_data = MapMetaData(resolution=0.1, width=7, height=7)
        return OccupancyGrid(info=meta_data, data=grid)

    def test_get_xy_map(self):
        self.map_data = MapData(self.get_occupancy_grid(), 65)
        self.assertEqual(self.map_data.get_xy_map(Point(x=0.0, y=0.0)), (0, 0))
        self.assertEqual(self.map_data.get_xy_map(Point(x=0.05, y=0.05)), (0, 0))
        self.assertEqual(self.map_data.get_xy_map(Point(x=0.051, y=0.051)), (1, 1))
        self.assertEqual(
            self.map_data.get_xy_map(Point(x=0.15, y=0.15)), (1, 1)
        )  # This occurs due to the floating point error
        self.assertEqual(self.map_data.get_xy_map(Point(x=0.151, y=0.151)), (2, 2))
        self.assertEqual(self.map_data.get_xy_map(Point(x=0.49, y=0.49)), (5, 5))
        self.assertEqual(self.map_data.get_xy_map(Point(x=0.3, y=0.3)), (3, 3))
        self.assertEqual(self.map_data.get_xy_map(Point(x=0.4, y=0.4)), (4, 4))
        self.assertEqual(self.map_data.get_xy_map(Point(x=0.9, y=0.2)), (9, 2))

    def test_get_index(self):
        self.map_data = MapData(self.get_occupancy_grid(), 65)
        self.assertEqual(self.map_data.get_index(Point(x=0.0, y=0.0)), 0)
        self.assertEqual(self.map_data.get_index(Point(x=0.05, y=0.0)), 0)
        self.assertEqual(self.map_data.get_index(Point(x=0.051, y=0.0)), 1)
        self.assertEqual(self.map_data.get_index(Point(x=0.1, y=0.0)), 1)
        self.assertEqual(self.map_data.get_index(Point(x=0.2, y=0.0)), 2)
        self.assertEqual(self.map_data.get_index(Point(x=0.0, y=0.1)), 7)
        self.assertEqual(self.map_data.get_index(Point(x=0.1, y=0.2)), 15)
        self.assertEqual(self.map_data.get_index(Point(x=0.2, y=0.4)), 30)

    def test_get_occupancy(self):
        self.map_data = MapData(self.get_occupancy_grid_v2(), 65)
        self.assertEqual(self.map_data.get_occupancy(0), 100)
        self.assertEqual(self.map_data.get_occupancy(1), 67)
        self.assertEqual(self.map_data.get_occupancy(7), 100)
        self.assertEqual(self.map_data.get_occupancy(8), 67)
        self.assertEqual(self.map_data.get_occupancy(16), 47)
        self.assertEqual(self.map_data.get_occupancy(48), 0)

    def test_is_free_cell(self):
        self.map_data = MapData(self.get_occupancy_grid_v2(), 65)
        self.assertEqual(self.map_data.is_free_cell(0), False)
        self.assertEqual(self.map_data.is_free_cell(1), False)
        self.assertEqual(self.map_data.is_free_cell(7), False)
        self.assertEqual(self.map_data.is_free_cell(16), True)
        self.assertEqual(self.map_data.is_free_cell(48), True)

    def test_is_unknown_cell(self):
        self.map_data = MapData(self.get_occupancy_grid_v2(), 65)
        self.assertEqual(self.map_data.is_unknown_cell(0), False)
        self.assertEqual(self.map_data.is_unknown_cell(35), True)


if __name__ == "__main__":
    unittest.main()
