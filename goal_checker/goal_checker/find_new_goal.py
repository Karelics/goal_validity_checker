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

from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid


class FindNewGoal:
    def __init__(self, occupancy_grid: OccupancyGrid):

        self.map_info = occupancy_grid.info

    def along_x(self, new_index: int, y_map: int, orig_goal_pose: Point) -> Point:
        """
        Returns new goal pose coordinates found in the x-direction
        """
        new_x = (
            (new_index - (self.map_info.width * y_map)) * self.map_info.resolution
        ) + self.map_info.origin.position.x
        new_y = orig_goal_pose.y
        return Point(x=new_x, y=new_y)

    def along_y(self, new_index: int, x_map: int, orig_goal_pose: Point) -> Point:
        """
        Returns new goal pose coordinates found in the y-direction
        """
        new_x = orig_goal_pose.x
        new_y = (
            ((new_index - x_map) / self.map_info.width) * self.map_info.resolution
        ) + self.map_info.origin.position.y
        return Point(x=new_x, y=new_y)

    def along_x_and_y(self, new_index: int) -> Point:
        """
        Returns new goal pose coordinates
        """
        new_y_cell = int(new_index / self.map_info.width)
        new_y = (
            new_y_cell * self.map_info.resolution
        ) + self.map_info.origin.position.y
        new_x = (
            (new_index - (self.map_info.width * new_y_cell)) * self.map_info.resolution
        ) + self.map_info.origin.position.x
        return Point(x=new_x, y=new_y)
