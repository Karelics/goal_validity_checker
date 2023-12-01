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

#ifndef BT_SRV_NODE_HPP_
#define BT_SRV_NODE_HPP_

#include <string>
#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/action_node.h"

#include "nav2_behavior_tree/bt_service_node.hpp"
#include "nav2_behavior_tree/bt_utils.hpp"
#include "nav2_util/node_utils.hpp"

#include "nav2_msgs/srv/clear_entire_costmap.hpp"
#include "nav2_msgs/srv/clear_costmap_around_robot.hpp"
#include "nav2_msgs/srv/clear_costmap_except_region.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"

#include "goal_checker_msgs/srv/goal_checker.hpp"



namespace nav2_behavior_tree
{
/**
 * @brief A nav2_behavior_tree::BtServiceNode class that wraps nav2_msgs::srv::GoalChecker
 */

class GoalCheckerService : public BtServiceNode<goal_checker_msgs::srv::GoalChecker>
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::GoalCheckerService
   * @param service_node_name Service name this node creates a client for
   * @param conf BT node configuration
   */
  GoalCheckerService(
    const std::string & service_node_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief The main override required by a BT service
   * @return BT::NodeStatus Status of tick execution
   */

  void on_tick() override;
  BT::NodeStatus on_completion(std::shared_ptr<goal_checker_msgs::srv::GoalChecker::Response> response) override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<geometry_msgs::msg::PoseStamped>(
          "goal", 
          "Original goal pose"),
        
        BT::OutputPort<geometry_msgs::msg::PoseStamped>(
          "new_goal", 
          "New goal pose"),
      });
  }
};
}  // namespace nav2_behavior_tree

#endif  // BT_SRV_NODE_HPP_