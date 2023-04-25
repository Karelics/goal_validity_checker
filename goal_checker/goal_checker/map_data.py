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

import numpy as np
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid

from typing import Tuple


class MapData:
    def __init__(self, occupancy_grid: OccupancyGrid, free_thresh: int):

        self.map_info = occupancy_grid.info
        self.costmap = np.array(occupancy_grid.data, dtype=np.uint8)
        self.set_unknown_area_search = True
        self.index = None
        self.free_thresh = free_thresh

    def get_xy_map(self, orig_goal_pose: Point) -> Tuple[int, int]:
        """
        Returns map coordinates (based on the resolution and origin of the map) of the original goal pose
        Known Issue: Bit representation floating point error
        """
        x_map = round(
            (orig_goal_pose.x - self.map_info.origin.position.x)
            / self.map_info.resolution
        )
        y_map = round(
            (orig_goal_pose.y - self.map_info.origin.position.y)
            / self.map_info.resolution
        )
        return x_map, y_map

    def get_index(self, orig_goal_pose: Point) -> int:
        """
        Returns index of the costmap array, corresponding to the original goal pose index
        """
        x_map, y_map = self.get_xy_map(orig_goal_pose)
        self.index = x_map + (self.map_info.width * y_map)
        return self.index

    def get_occupancy(self, index: int) -> int:
        """
        Returns occupancy of a corrresponding index of the costmap array
        """

        return self.costmap[index]

    def is_free_cell(self, index: int) -> bool:
        """
        Checks if an index of the costmap array is unoccupied
        """
        return (
            0 <= index < len(self.costmap)
            and self.get_occupancy(index) < self.free_thresh
        )

    def is_unknown_cell(self, index: int) -> bool:
        """
        Checks if an index of the costmap array is unknown
        """
        return self.get_occupancy(index) == 255

    def is_goal_pose_out_of_boundaries(self, orig_goal_pose: Point) -> bool:
        """
        Checks if origianl goal pose is within the map
        """
        end_x = self.map_info.origin.position.x + (
            self.map_info.width * self.map_info.resolution
        )
        end_y = self.map_info.origin.position.y + (
            self.map_info.height * self.map_info.resolution
        )
        return (
            orig_goal_pose.x < self.map_info.origin.position.x
            or orig_goal_pose.x > end_x
            or orig_goal_pose.y < self.map_info.origin.position.y
            or orig_goal_pose.y > end_y
        )
