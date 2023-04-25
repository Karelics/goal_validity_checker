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


import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    goal_validity_checker_params = os.path.join(
        get_package_share_directory("goal_checker"),
        "param",
        "goal_validity_checker.yaml",
    )

    return LaunchDescription(
        [
            # Setting sim_time as first thing in launch_list, so that it will be automatically set for all the nodes
            # SetParameter(name="use_sim_time", value=True),
            Node(
                package="goal_checker",
                executable="goal_validity_checker.py",
                emulate_tty=True,
                parameters=[goal_validity_checker_params],
                output={"both": {"screen", "log", "own_log"}},
                arguments=["--ros-args", "--log-level", "INFO"],
            ),
        ]
    )
