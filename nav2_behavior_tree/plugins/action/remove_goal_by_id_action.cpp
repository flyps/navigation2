// Copyright (c) 2025 Flyps

#include <string>
#include <memory>

#include "nav2_util/geometry_utils.hpp"

#include "nav2_behavior_tree/plugins/action/remove_goal_by_id_action.hpp"

namespace nav2_behavior_tree {

    RemoveGoalById::RemoveGoalById(
            const std::string &name,
            const BT::NodeConfiguration &conf)
            : BT::ActionNodeBase(name, conf),
              goal_to_remove_id_(-1) {}

    void RemoveGoalById::initialize() {
        getInput("goal_to_remove_id", goal_to_remove_id_);
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    }

    inline BT::NodeStatus RemoveGoalById::tick() {
        if (!BT::isStatusActive(status())) {
            initialize();
        }

        nav_msgs::msg::Goals goal_poses;
        getInput("input_goals", goal_poses);

        // Check if the goal to remove is valid
        auto goals_size_ = static_cast<int>(goal_poses.goals.size());
        if (goals_size_ == 0 || goals_size_ == 1) {
            RCLCPP_WARN(node_->get_logger(), "No goals to remove. Must be at least 2 goals to remove one.");
            setOutput("output_goals", goal_poses);
            return BT::NodeStatus::SUCCESS;
        } else {
            if (goal_to_remove_id_ < 0) {
                auto new_id = goals_size_ - goal_to_remove_id_;
                if (new_id < 0) {
                    RCLCPP_WARN(node_->get_logger(), "Goal ID %d is out of range", goal_to_remove_id_);
                    setOutput("output_goals", goal_poses);
                    return BT::NodeStatus::FAILURE;
                } else {
                    goal_to_remove_id_ = new_id;
                }
            } else if (goal_to_remove_id_ >= goals_size_) {
                RCLCPP_WARN(node_->get_logger(), "Goal ID %d is out of range", goal_to_remove_id_);
                setOutput("output_goals", goal_poses);
                return BT::NodeStatus::FAILURE;
            }
        }

        // Remove the goal with the specified ID
        nav_msgs::msg::Goals goal_poses_without_one;
        goal_poses_without_one.header = goal_poses.header;
        goal_poses_without_one.goals.reserve(goal_poses.goals.size() - 1);
        for (size_t i = 0; i < goal_poses.goals.size(); ++i) {
            if (static_cast<int>(i) != goal_to_remove_id_) {
                goal_poses_without_one.goals.push_back(goal_poses.goals[i]);
            }
        }

        setOutput("output_goals", goal_poses_without_one);

        return BT::NodeStatus::SUCCESS;
    }

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::RemoveGoalById>("RemoveGoalById");
}