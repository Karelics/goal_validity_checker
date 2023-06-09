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


import rclpy
from rclpy.node import Node

from rcl_interfaces.msg import ParameterDescriptor
from rclpy.parameter import ParameterType

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
from goal_checker_msgs.srv import GoalChecker

from goal_checker.search import Search


class GoalCheckerService(Node):
    def __init__(self):
        super().__init__("goal_checker")
        self.srv = self.create_service(
            GoalChecker, "goal_checker_validity", self.goal_checker_callback
        )
        self._sub_gcm = self.create_subscription(
            OccupancyGrid, "/global_costmap/costmap", self.listener_callback, 10
        )
        max_range_descriptor = ParameterDescriptor(
            name="max_range_new_goal",
            type=ParameterType.PARAMETER_DOUBLE,
            description="Maximum range allowed to search for new goal pose",
        )
        unknown_search_descriptor = ParameterDescriptor(
            name="set_unknown_area_search",
            type=ParameterType.PARAMETER_BOOL,
            description="Allow searching for new goal pose if original one is unknown",
        )
        free_thresh_descriptor = ParameterDescriptor(
            name="free_thresh",
            type=ParameterType.PARAMETER_INTEGER,
            description="Threshold determinining free occupancy cells in the global costmap",
        )
        min_range_descriptor = ParameterDescriptor(
            name="min_range",
            type=ParameterType.PARAMETER_DOUBLE,
            description="Minimum range from new goal pose found to avoid the need to calculate new poses due to laser noise",
        )
        self.declare_parameter("max_range_new_goal", 2.0, max_range_descriptor)
        self.declare_parameter("free_thresh", 65, free_thresh_descriptor)
        self.declare_parameter(
            "set_unknown_area_search", False, unknown_search_descriptor
        )
        self.declare_parameter("min_range", 0.1, min_range_descriptor)
        self.occupancy_grid = None

    def listener_callback(self, msg: OccupancyGrid) -> None:
        self.occupancy_grid = msg

    def goal_checker_callback(
        self, request: GoalChecker.Request, response: GoalChecker.Response
    ) -> GoalChecker.Response:

        response.new_goal_pose.header.frame_id = request.goal_pose.header.frame_id
        response.new_goal_pose.pose.orientation = request.goal_pose.pose.orientation
        orig_goal_pose = request.goal_pose.pose.position

        max_range_new_goal = self.get_parameter("max_range_new_goal").value
        free_thresh = self.get_parameter("free_thresh").value
        set_unknown_area_search = self.get_parameter("set_unknown_area_search").value
        min_range = self.get_parameter("min_range").value

        if self.occupancy_grid.info.resolution <= 0:
            response.success = False
            response.message = "Resolution is invalid, it has to be a positive value"
            return response

        search = Search(
            self.occupancy_grid,
            max_range_new_goal,
            free_thresh,
            set_unknown_area_search,
            min_range,
        )

        result = search.check_for_goal_pose(orig_goal_pose)

        if result.state.value == "VALID":
            response.new_goal_pose.pose.position = orig_goal_pose
            response.success = True
            response.message = "Original valid pose"
            self.get_logger().info("NO OBSTACLES CAN SAFELY NAVIGATE")

        elif result.state.value == "UNKNOWN":
            response.new_goal_pose.pose = Pose()
            response.success = False
            response.message = "Unkown area"
            self.get_logger().error("Unknown area please select a VALID goal pose")

        elif result.state.value == "NEW_GOAL_FOUND":
            response.new_goal_pose.pose.position = result.goal
            response.success = True
            response.message = "New goal pose found"
            self.get_logger().info(
                "NEW GOAL FOUND at: (%.2f, %.2f)" % (result.goal.x, result.goal.y)
            )

        elif result.state.value == "OUT_OF_RANGE":
            response.new_goal_pose.pose = Pose()
            response.success = False
            response.message = "No goal found within the defined range"
            self.get_logger().error(
                "No goal found within the defined range of: %.2f meters"
                % max_range_new_goal
            )

        else:
            response.success = False
            response.message = "Got undefined state for the new goal"

        return response


def main():
    rclpy.init()
    goal_checker_service = GoalCheckerService()
    try:
        rclpy.spin(goal_checker_service)
    except KeyboardInterrupt:
        pass
    goal_checker_service.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
