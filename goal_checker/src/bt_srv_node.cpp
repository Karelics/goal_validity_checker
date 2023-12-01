// Goal Validity Checker - ROS 2 Node checking valid goal poses during navigation.
// Copyright (C) 2023  Karelics Oy

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program. If not, see <https://www.gnu.org/licenses/>.


#include <string>
#include <memory>
#include "goal_checker/bt_srv_node.hpp"

namespace nav2_behavior_tree
{

GoalCheckerService::GoalCheckerService(
  const std::string & service_node_name,
  const BT::NodeConfiguration & conf)
: BtServiceNode<goal_checker_msgs::srv::GoalChecker>(service_node_name, conf){}


void GoalCheckerService::on_tick()
{
    getInput("goal", request_->goal_pose);
}

BT::NodeStatus GoalCheckerService::on_completion(std::shared_ptr<goal_checker_msgs::srv::GoaChecker::Response> response)
{
    if (response->success) {
      setOutput("new_goal", response->new_goal_pose);
      return BT::NodeStatus::SUCCESS;
    } else {
      return BT::NodeStatus::FAILURE;
    }
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_file_logger.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::GoalCheckerService>("GoalCheckerService");
}


