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

import math

from enum import Enum
from dataclasses import dataclass
from typing import Optional, Tuple, List

from goal_checker.map_data import MapData
from goal_checker.find_new_goal import FindNewGoal

from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid


class State(Enum):
    UNKNOWN = "UNKNOWN"
    VALID = "VALID"
    OUT_OF_RANGE = "OUT_OF_RANGE"
    NEW_GOAL_FOUND = "NEW_GOAL_FOUND"


@dataclass
class Result:
    state: State
    goal: Point


class Search:
    def __init__(
        self,
        occupancy_grid: OccupancyGrid,
        max_range_new_goal: float,
        free_thresh: int,
        set_unknown_area_search: bool,
        min_range: float,
    ):
        self.weight = 1
        self.weight_diag = 1
        self.weight_min_range = 0
        self.weight_diag_min_range = 0
        self.cell_list = []
        self.cell_list_min_range = []
        self.max_range_new_goal = max_range_new_goal
        self.set_unknown_area_search = set_unknown_area_search
        self.min_range = min_range
        self.index = None
        self.map_data = MapData(occupancy_grid, free_thresh)
        self.find_new_goal = FindNewGoal(occupancy_grid)
        self.width = self.map_data.map_info.width

    def _get_cell_list(self, weight: int, weight_diag: int) -> List[int]:
        cell_list = [
            weight,
            -weight,
            self.width * weight,
            -self.width * weight,
            (self.width - 1) * weight_diag,
            (self.width + 1) * weight_diag,
            -(self.width + 1) * weight_diag,
            -(self.width - 1) * weight_diag,
        ]
        return cell_list

    def _cell_list_right_search(self) -> None:
        self.cell_list = [
            self.weight,
            self.width * self.weight,
            -self.width * self.weight,
            (self.width + 1) * self.weight_diag,
            -(self.width - 1) * self.weight_diag,
        ]

    def _cell_list_left_search(self) -> None:
        self.cell_list = [
            -self.weight,
            self.width * self.weight,
            -self.width * self.weight,
            (self.width - 1) * self.weight_diag,
            -(self.width + 1) * self.weight_diag,
        ]

    def _calculate_index(self, orig_goal_pose: Point) -> None:
        self.index = self.map_data.get_index(orig_goal_pose)

    def _is_goal_pose_at_bc(self) -> bool:
        goal_pose_at_bc = False

        if len(self.cell_list) == 8 and self.index % self.width == 0:
            goal_pose_at_bc = True
            self._cell_list_right_search()

        elif len(self.cell_list) == 8 and self.index % self.width == self.width - 1:
            goal_pose_at_bc = True
            self._cell_list_left_search()

        return goal_pose_at_bc

    def _is_boundary_left_met(self, neighbor_cell_offset: int) -> bool:

        if (self.index + neighbor_cell_offset) % self.width == 0:  # right boundary cdt
            return True
        return False

    def _is_boundary_right_met(self, neighbor_cell_offset: int) -> bool:

        if (
            self.index + neighbor_cell_offset
        ) % self.width == self.width - 1:  # right boundary cdt
            return True
        return False

    def _check_in_defined_range(self, max_weight: int) -> bool:
        return self.weight < max_weight

    def _update_cell_list(self) -> None:

        if len(self.cell_list) == 8:  # access self.cell_list = _get_cell_list()
            self.cell_list = self._get_cell_list(self.weight, self.weight_diag)

        elif self.cell_list[0] > 0:  # access self.cell_list = _cell_list_right_search()
            self._cell_list_right_search()

        elif self.cell_list[0] < 0:  # access self.cell_list = _cell_list_left_search()
            self._cell_list_left_search()

    def _expand(self, max_weight: int) -> bool:
        self.weight += 1
        self.weight_diag += 1
        self._update_cell_list()

        return self._check_in_defined_range(max_weight)

    def _get_max_weights(self) -> Tuple[int, int]:
        max_weight = (
            int(self.max_range_new_goal / self.map_data.map_info.resolution) + 1
        )

        max_weight_diag = int(
            self.max_range_new_goal / (math.sqrt(2) * self.map_data.map_info.resolution)
        )
        return max_weight, max_weight_diag

    def _get_min_range_weights(self) -> int:
        if self.min_range == 0:
            min_weight = 0
        elif self.min_range < self.map_data.map_info.resolution:
            min_weight = 1
        else:
            min_weight = math.ceil(self.min_range / self.map_data.map_info.resolution)

        return min_weight

    def _left_or_right_unoccupied(self, neighbor_cell_offset: int) -> bool:
        return neighbor_cell_offset in [self.weight, -self.weight]

    def _upper_or_lower_unoccupied(self, neighbor_cell_offset: int) -> bool:
        return neighbor_cell_offset in [
            self.width * self.weight,
            -self.width * self.weight,
        ]

    def _diagonal_unoccupied(
        self, neighbor_cell_offset: int, max_weight_diag: int, weight_diag: int
    ) -> bool:
        index_on_diagonal = neighbor_cell_offset in [
            (self.width - 1) * weight_diag,
            (self.width + 1) * weight_diag,
            -(self.width + 1) * weight_diag,
            -(self.width - 1) * weight_diag,
        ]
        return index_on_diagonal and weight_diag <= max_weight_diag

    def _find_index_on_right(
        self, neighbor_cell_offset: int, min_weight: int
    ) -> Optional[int]:

        new_index = None
        found_new_index = True

        # find NEW INDEX on RIGHT SIDE
        for i in range(1, min_weight + 2):
            index_right = self.index + neighbor_cell_offset + i
            if not self.map_data.is_free_cell(index_right):
                found_new_index = False
                break
            if index_right % self.width == self.width - 1:
                found_new_index = False
                break

        if found_new_index is True:
            new_index = self.index + neighbor_cell_offset + min_weight + 1
        return new_index

    def _find_index_on_left(
        self, neighbor_cell_offset: int, min_weight: int
    ) -> Optional[int]:

        new_index = None
        found_new_index = True

        # find NEW INDEX on LEFT SIDE
        for i in range(1, min_weight + 2):
            index_left = self.index + neighbor_cell_offset - i

            if not self.map_data.is_free_cell(index_left):
                found_new_index = False
                break
            if index_left % self.width == 0:
                found_new_index = False
                break

        if found_new_index is True:
            new_index = self.index + neighbor_cell_offset - min_weight - 1
        return new_index

    def _find_index_top(
        self, neighbor_cell_offset: int, min_weight: int
    ) -> Optional[int]:

        new_index = None
        found_new_index = True

        # find NEW INDEX in UPPER DIRECTION
        for i in range(1, min_weight + 2):
            index_top = self.index + neighbor_cell_offset + (i * self.width)

            if not self.map_data.is_free_cell(index_top):
                found_new_index = False
                break
            if (
                self.width * (self.map_data.map_info.height - 1)
                <= index_top
                < self.width * self.map_data.map_info.height
            ):
                found_new_index = False
                break
        if found_new_index is True:
            new_index = (
                self.index + neighbor_cell_offset + ((min_weight + 1) * self.width)
            )
        return new_index

    def _find_index_bottom(
        self, neighbor_cell_offset: int, min_weight: int
    ) -> Optional[int]:

        new_index = None
        found_new_index = True

        # find NEW INDEX in LOWER DIRECTION
        for i in range(1, min_weight + 2):
            index_bottom = self.index + neighbor_cell_offset - (i * self.width)

            if not self.map_data.is_free_cell(index_bottom):
                found_new_index = False
                break

            if 0 <= index_bottom < self.width:
                found_new_index = False
                break

        if found_new_index is True:
            new_index = (
                self.index + neighbor_cell_offset - ((min_weight + 1) * self.width)
            )
        return new_index

    def _find_new_index(self, neighbor_cell_offset: int) -> Optional[int]:
        return self.index + neighbor_cell_offset
        min_weight = self._get_min_range_weights()
        right_offset = [
            self.weight,
            (self.width + 1) * self.weight_diag,
            -(self.width - 1) * self.weight_diag,
        ]
        left_offset = [
            -self.weight,
            (self.width - 1) * self.weight_diag,
            -(self.width + 1) * self.weight_diag,
        ]
        top_offset = self.width * self.weight
        bottom_offset = -self.width * self.weight

        # first index found on the RIGHT SIDE
        if neighbor_cell_offset in right_offset:
            new_index = self._find_index_on_right(neighbor_cell_offset, min_weight)

        # first index found on the LEFT SIDE
        elif neighbor_cell_offset in left_offset:
            new_index = self._find_index_on_left(neighbor_cell_offset, min_weight)

        # first index found in UPPER DIRECTION
        elif neighbor_cell_offset == top_offset:
            new_index = self._find_index_top(neighbor_cell_offset, min_weight)

        # first index found in LOWER DIRECTION
        elif neighbor_cell_offset == bottom_offset:
            new_index = self._find_index_bottom(neighbor_cell_offset, min_weight)

        return new_index

    def _min_range_search(self, new_index: int) -> bool:
        min_weight = self._get_min_range_weights()
        free_neighbor_cells = True

        self.cell_list_min_range = self._get_cell_list(
            self.weight_min_range, self.weight_diag_min_range
        )

        for j in range(min_weight):
            self.weight_min_range += 1
            self.weight_diag_min_range += 1
            for k in self._get_cell_list(
                self.weight_min_range, self.weight_diag_min_range
            ):
                # Check if cell is occupied -> Stop search as goal cannot be achieved
                if not self.map_data.is_free_cell(new_index + k):
                    free_neighbor_cells = False
                    break

                bc_left = (new_index + k) % self.width == 0
                bc_right = (new_index + k) % self.width == self.width - 1
                bc_bottom = 0 <= new_index + k < self.width
                bc_top = (
                    self.width * (self.map_data.map_info.height - 1)
                    <= new_index + k
                    < self.width * self.map_data.map_info.height
                )

                bc = bc_left or bc_right or bc_bottom or bc_top

                if not self.map_data.is_free_cell(
                    new_index
                ) or not self.map_data.is_free_cell(new_index + k):
                    free_neighbor_cells = False
                    break

                if bc and j != min_weight - 1:
                    free_neighbor_cells = False
                    break

            if free_neighbor_cells is False:
                self.weight_min_range = 0
                self.weight_diag_min_range = 0
                break

        return free_neighbor_cells

    def _find_new_goal_x_axis(
        self,
        neighbor_cell_offset: int,
        min_weight: int,
        orig_goal_pose: Point,
        y_map: int,
    ) -> Tuple[Optional[Point], bool]:

        new_goal_found = False
        new_pose = None

        if min_weight == 0:
            new_index = self.index + neighbor_cell_offset
            new_pose = self.find_new_goal.along_x(new_index, y_map, orig_goal_pose)
            new_goal_found = True

        else:
            new_index = self._find_new_index(neighbor_cell_offset)

            if new_index or new_index == 0:
                if self._min_range_search(new_index):
                    new_pose = self.find_new_goal.along_x(
                        new_index, y_map, orig_goal_pose
                    )
                    new_goal_found = True
        return new_pose, new_goal_found

    def _find_new_goal_y_axis(
        self,
        neighbor_cell_offset: int,
        min_weight: int,
        orig_goal_pose: Point,
        x_map: int,
    ) -> Tuple[Optional[Point], bool]:

        new_goal_found = False
        new_pose = None

        if min_weight == 0:
            new_index = self.index + neighbor_cell_offset
            new_pose = self.find_new_goal.along_y(new_index, x_map, orig_goal_pose)
            new_goal_found = True

        else:
            new_index = self._find_new_index(neighbor_cell_offset)

            if new_index or new_index == 0:
                if self._min_range_search(new_index):
                    new_pose = self.find_new_goal.along_y(
                        new_index, x_map, orig_goal_pose
                    )
                    new_goal_found = True
        return new_pose, new_goal_found

    def _find_new_goal_diag(
        self,
        neighbor_cell_offset: int,
        min_weight: int,
    ) -> Tuple[Optional[Point], bool]:

        new_goal_found = False
        new_pose = None

        if min_weight == 0:
            new_index = self.index + neighbor_cell_offset
            new_pose = self.find_new_goal.along_x_and_y(new_index)
            new_goal_found = True

        else:
            new_index = self._find_new_index(neighbor_cell_offset)

            if new_index or new_index == 0:
                if self._min_range_search(new_index):
                    new_pose = self.find_new_goal.along_x_and_y(new_index)
                    new_goal_found = True
        return new_pose, new_goal_found

    def _search_neighbor_cells(
        self,
        neighbor_cell_offset: int,
        orig_goal_pose: Point,
        max_weight_diag: int,
    ) -> Tuple[Optional[Point], bool]:

        # Check if the cell we are about to check is free
        if not self.map_data.is_free_cell(
            self.index + neighbor_cell_offset
        ):  # Cell index is not index it is more an offset
            return None, False
        new_pose: Optional[Point] = None
        new_goal_found = False
        x_map, y_map = self.map_data.get_xy_map(orig_goal_pose)
        min_weight = self._get_min_range_weights()

        if self._left_or_right_unoccupied(neighbor_cell_offset):
            new_pose, new_goal_found = self._find_new_goal_x_axis(
                neighbor_cell_offset, min_weight, orig_goal_pose, y_map
            )

        elif self._upper_or_lower_unoccupied(neighbor_cell_offset):
            new_pose, new_goal_found = self._find_new_goal_y_axis(
                neighbor_cell_offset, min_weight, orig_goal_pose, x_map
            )

        elif self._diagonal_unoccupied(
            neighbor_cell_offset, max_weight_diag, self.weight_diag
        ):
            new_pose, new_goal_found = self._find_new_goal_diag(
                neighbor_cell_offset, min_weight
            )

        return new_pose, new_goal_found

    def _new_goal_pose(self, orig_goal_pose: Point) -> Tuple[Optional[Point], bool]:

        max_weight, max_weight_diag = self._get_max_weights()
        new_pose = None
        in_defined_range = True
        new_goal_found = False
        bc_left_met = False
        bc_right_met = False
        goal_pose_at_bc = False

        self.cell_list = self._get_cell_list(self.weight, self.weight_diag)
        while new_goal_found is False and in_defined_range is True:

            for neighbor_cell_offset in self.cell_list:
                goal_pose_at_bc = self._is_goal_pose_at_bc()
                if goal_pose_at_bc:
                    break

                if not goal_pose_at_bc and self._is_boundary_left_met(
                    neighbor_cell_offset
                ):  # Has side effect -> Creates new cell_list
                    bc_left_met = True
                elif not goal_pose_at_bc and self._is_boundary_right_met(
                    neighbor_cell_offset
                ):  # Has side effect -> Creates new cell_list
                    bc_right_met = True

                new_pose, new_goal_found = self._search_neighbor_cells(
                    neighbor_cell_offset,
                    orig_goal_pose,
                    max_weight_diag,
                )
                if new_goal_found:
                    break

            if goal_pose_at_bc is True:
                self._update_cell_list()
                goal_pose_at_bc = False
                continue
            if (bc_left_met or bc_right_met) and not new_goal_found:
                if bc_left_met:
                    print("BOUNDARY CONDITION LEFT SIDE --> SEARCH ONLY ON RIGHT SIDE")
                    self._cell_list_right_search()
                elif bc_right_met:
                    print("BOUNDARY CONDITION RIGHT SIDE --> SEARCH ONLY ON LEFT SIDE")
                    self._cell_list_left_search()

            if not new_goal_found:
                in_defined_range = self._expand(max_weight)
        is_valid_goal = new_goal_found and in_defined_range

        return new_pose, is_valid_goal

    def check_for_goal_pose(self, orig_goal_pose: Point) -> Result:
        """
        Checks if the goal pose is valid for the path planner.

        :param orig_goal_pose: corresponds to the x and y coordinates of the goal pose

        :return Result: Composed of 2 objects which are (i) the state of the corresponding goal pose and (ii) the goal pose coordinates

            (i) We have 4 possible states:
                1) UNKNOWN: Original goal pose is located at unknown area and the self.set_unknown_area_search parameter is set to False
                2) VALID: Original goal pose is valid
                3) OUT_OF_RANGE: Goal pose is out of defined range
                4) NEW_GOAL_FOUND: New goal pose found

            (ii) The goal pose coordinates can either be:
                1) The same as the original one for the VALID case
                2) A new set of coordinates for the NEW_GOAL_FOUND case
                3) An empty Point message for the UNKNOWN and OUT_OF_RANGE cases

        """

        self._calculate_index(orig_goal_pose)

        if self.map_data.is_free_cell(self.index):
            free_neighbor_cells = self._min_range_search(self.index)
            if free_neighbor_cells is True:
                return Result(state=State.VALID, goal=orig_goal_pose)
            else:
                new_pose, valid_goal = self._new_goal_pose(orig_goal_pose)

                if valid_goal:
                    return Result(state=State.NEW_GOAL_FOUND, goal=new_pose)
                else:
                    return Result(state=State.OUT_OF_RANGE, goal=Point())
        elif (
            self.map_data.is_unknown_cell(self.index)
            and self.set_unknown_area_search is False
        ):
            return Result(state=State.UNKNOWN, goal=Point())
        else:
            new_pose, valid_goal = self._new_goal_pose(orig_goal_pose)

            if valid_goal:
                return Result(state=State.NEW_GOAL_FOUND, goal=new_pose)
            else:
                return Result(state=State.OUT_OF_RANGE, goal=Point())
