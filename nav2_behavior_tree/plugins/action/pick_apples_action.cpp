// Copyright (c) 2025 Flyps

#include <string>
#include <memory>
#include <cmath>

#include "nav2_behavior_tree/plugins/action/pick_apples_action.hpp"

namespace nav2_behavior_tree {

    PickApplesAction::PickApplesAction(
            const std::string &xml_tag_name,
            const std::string &action_name,
            const BT::NodeConfiguration &conf)
            : BtActionNode<nav2_msgs::action::Wait>(xml_tag_name, action_name, conf) {
    }

    void PickApplesAction::initialize() {
        double timeout;
        getInput("timeout", timeout);
        if (timeout <= 0) {
            RCLCPP_WARN_ONCE(
                    node_->get_logger(), "Timeout cannot be less than or zero. Setting to 120 seconds.");
            timeout = 120.0;
        }

        goal_.time = rclcpp::Duration::from_seconds(timeout);
    }

    void PickApplesAction::on_tick() {
        if (!BT::isStatusActive(status())) {
            initialize();
        }
    }

    BT::NodeStatus PickApplesAction::on_success() {
        RCLCPP_INFO(node_->get_logger(), "Apples picked successfully.");
        setOutput("error_code_id", ActionResult::NONE);
        setOutput("error_msg", "");
        return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus PickApplesAction::on_aborted() {
        RCLCPP_ERROR(node_->get_logger(), "Picking apples action aborted.");
        setOutput("error_code_id", result_.result->error_code);
        setOutput("error_msg", result_.result->error_msg);
        return BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus PickApplesAction::on_cancelled() {
        RCLCPP_INFO(node_->get_logger(), "Picking apples action cancelled.");
        setOutput("error_code_id", ActionResult::NONE);
        setOutput("error_msg", "");
        return BT::NodeStatus::SUCCESS;
    }

    void PickApplesAction::on_timeout() {
        RCLCPP_ERROR(node_->get_logger(), "Picking apples action timed out.");
        setOutput("error_code_id", ActionResult::TIMEOUT);
        setOutput("error_msg", "Behavior Tree action client timed out waiting.");
    }

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory) {
    BT::NodeBuilder builder =
            [](const std::string &name, const BT::NodeConfiguration &config) {
                return std::make_unique<nav2_behavior_tree::PickApplesAction>(name, "pick_apples", config);
            };

    factory.registerBuilder<nav2_behavior_tree::PickApplesAction>("PickApples", builder);
}
