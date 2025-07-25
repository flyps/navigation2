// Copyright (c) 2025 Flyps

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__REMOVE_GOAL_BY_ID_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__REMOVE_GOAL_BY_ID_ACTION_HPP_

#include <vector>
#include <memory>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/json_export.h"
#include "nav_msgs/msg/goals.hpp"
#include "nav2_behavior_tree/bt_utils.hpp"
#include "nav2_behavior_tree/json_utils.hpp"
#include "nav2_util/geometry_utils.hpp"


namespace nav2_behavior_tree {

/**
 * @brief A BT::ActionNodeBase that removes a Goal from a list of goals based on its ID.
 * @note This is an Asynchronous node. It will re-initialize when halted.
 */
    class RemoveGoalById : public BT::ActionNodeBase {
    public:
        RemoveGoalById(
                const std::string &xml_tag_name,
                const BT::NodeConfiguration &conf);

        /**
         * @brief Function to read parameters and initialize class variables
         */
        void initialize();

        static BT::PortsList providedPorts() {
            // Register JSON definitions for the types used in the ports
            BT::RegisterJsonDefinition<nav_msgs::msg::Goals>();

            return {
                    BT::InputPort<nav_msgs::msg::Goals>(
                            "input_goals",
                            "Original goals to remove a goal from"),
                    BT::InputPort<double>(
                            "goal_to_remove_id", -1,
                            "ID of the goal to remove from the list of goals"),
                    BT::OutputPort<nav_msgs::msg::Goals>(
                            "output_goals",
                            "Goals with the goal removed")
            };
        }

    private:
        void halt() override {}

        BT::NodeStatus tick() override;

        int goal_to_remove_id_;
        rclcpp::Node::SharedPtr node_;
    };

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__REMOVE_GOAL_BY_ID_ACTION_HPP_
