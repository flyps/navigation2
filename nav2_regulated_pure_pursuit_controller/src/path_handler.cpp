// Copyright (c) 2022 Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <string>
#include <limits>
#include <memory>
#include <vector>
#include <utility>
#include <cmath>

#include "tf2/utils.h"
#include "nav2_regulated_pure_pursuit_controller/path_handler.hpp"
#include "nav2_core/controller_exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"

namespace nav2_regulated_pure_pursuit_controller
{

using nav2_util::geometry_utils::euclidean_distance;

PathHandler::PathHandler(
  tf2::Duration transform_tolerance,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
: transform_tolerance_(transform_tolerance), tf_(tf), costmap_ros_(costmap_ros)
{
}

double PathHandler::getCostmapMaxExtent() const
{
  const double max_costmap_dim_meters = std::max(
    costmap_ros_->getCostmap()->getSizeInMetersX(),
    costmap_ros_->getCostmap()->getSizeInMetersY());
  return max_costmap_dim_meters / 2.0;
}

nav_msgs::msg::Path PathHandler::transformGlobalPlan(
  const geometry_msgs::msg::PoseStamped & pose,
  double max_robot_pose_search_dist,
  bool reject_unit_path,
  bool prune_plan)
{
  // Use the inversion-aware path for transformation
  auto & plan_to_use = global_plan_up_to_inversion_;

  if (plan_to_use.poses.empty()) {
    throw nav2_core::InvalidPath("Received plan with zero length");
  }

  if (reject_unit_path && plan_to_use.poses.size() == 1) {
    throw nav2_core::InvalidPath("Received plan with length of one");
  }

  // let's get the pose of the robot in the frame of the plan
  geometry_msgs::msg::PoseStamped robot_pose;
  if (!transformPose(plan_to_use.header.frame_id, pose, robot_pose)) {
    throw nav2_core::ControllerTFError("Unable to transform robot pose into global plan's frame");
  }

  auto closest_pose_upper_bound =
    nav2_util::geometry_utils::first_after_integrated_distance(
    plan_to_use.poses.begin(), plan_to_use.poses.end(), max_robot_pose_search_dist);

  // First find the closest pose on the path to the robot
  // bounded by when the path turns around (if it does) so we don't get a pose from a later
  // portion of the path
  auto transformation_begin =
    nav2_util::geometry_utils::min_by(
    plan_to_use.poses.begin(), closest_pose_upper_bound,
    [&robot_pose](const geometry_msgs::msg::PoseStamped & ps) {
      return euclidean_distance(robot_pose, ps);
    });

  // Make sure we always have at least 2 points on the transformed plan and that we don't prune
  // the global plan below 2 points in order to have always enough point to interpolate the
  // end of path direction
  if (plan_to_use.poses.begin() != closest_pose_upper_bound && plan_to_use.poses.size() > 1 &&
    transformation_begin == std::prev(closest_pose_upper_bound))
  {
    transformation_begin = std::prev(std::prev(closest_pose_upper_bound));
  }

  // We'll discard points on the plan that are outside the local costmap
  const double max_costmap_extent = getCostmapMaxExtent();
  auto transformation_end = std::find_if(
    transformation_begin, plan_to_use.poses.end(),
    [&](const auto & global_plan_pose) {
      return euclidean_distance(global_plan_pose, robot_pose) > max_costmap_extent;
    });

  // Lambda to transform a PoseStamped from global frame to local
  auto transformGlobalPoseToLocal = [&](const auto & global_plan_pose) {
      geometry_msgs::msg::PoseStamped stamped_pose, transformed_pose;
      stamped_pose.header.frame_id = plan_to_use.header.frame_id;
      stamped_pose.header.stamp = rclcpp::Time(0);  // Use latest available transform
      stamped_pose.pose = global_plan_pose.pose;
      if (!transformPose(costmap_ros_->getBaseFrameID(), stamped_pose, transformed_pose)) {
        throw nav2_core::ControllerTFError("Unable to transform plan pose into local frame");
      }
      transformed_pose.pose.position.z = 0.0;
      return transformed_pose;
    };

  // Transform the near part of the global plan into the robot's frame of reference.
  nav_msgs::msg::Path transformed_plan;
  std::transform(
    transformation_begin, transformation_end,
    std::back_inserter(transformed_plan.poses),
    transformGlobalPoseToLocal);
  transformed_plan.header.frame_id = costmap_ros_->getBaseFrameID();
  transformed_plan.header.stamp = robot_pose.header.stamp;

  // Remove the portion of the global plan that we've already passed so we don't
  // process it on the next iteration (this is called path pruning)
  if (prune_plan) {
    plan_to_use.poses.erase(begin(plan_to_use.poses), transformation_begin);
  }

  if (transformed_plan.poses.empty()) {
    throw nav2_core::InvalidPath("Resulting plan has 0 poses in it.");
  }

  return transformed_plan;
}

bool PathHandler::transformPose(
  const std::string frame,
  const geometry_msgs::msg::PoseStamped & in_pose,
  geometry_msgs::msg::PoseStamped & out_pose) const
{
  if (in_pose.header.frame_id == frame) {
    out_pose = in_pose;
    return true;
  }

  try {
    tf_->transform(in_pose, out_pose, frame, transform_tolerance_);
    out_pose.header.frame_id = frame;
    return true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(logger_, "Exception in transformPose: %s", ex.what());
  }
  return false;
}

nav_msgs::msg::Path::_poses_type::iterator
PathHandler::findFirstPathInversion(nav_msgs::msg::Path & plan)
{
  // Iterate through path to find the first inversion (cusp point)
  for (auto pose_it = plan.poses.begin(); pose_it != plan.poses.end() - 1; ++pose_it) {
    auto next_pose_it = pose_it + 1;

    // Calculate vectors between consecutive poses
    double dx1 = pose_it->pose.position.x - (pose_it > plan.poses.begin() ?
                 std::prev(pose_it)->pose.position.x : pose_it->pose.position.x);
    double dy1 = pose_it->pose.position.y - (pose_it > plan.poses.begin() ?
                 std::prev(pose_it)->pose.position.y : pose_it->pose.position.y);
    double dx2 = next_pose_it->pose.position.x - pose_it->pose.position.x;
    double dy2 = next_pose_it->pose.position.y - pose_it->pose.position.y;

    // Check for direction reversal using dot product
    double dot_product = dx1 * dx2 + dy1 * dy2;

    if (dot_product < 0.0) {
      return pose_it;
    }

    // Check for in-place rotation (overlapping points with different orientations)
    if (std::hypot(dx2, dy2) < 1e-4) {
      double yaw_curr = tf2::getYaw(pose_it->pose.orientation);
      double yaw_next = tf2::getYaw(next_pose_it->pose.orientation);
      if (std::abs(shortest_angular_distance(yaw_curr, yaw_next)) > 0.1) {
        return pose_it;
      }
    }
  }

  return plan.poses.end();
}

void PathHandler::removePosesAfterFirstInversion(nav_msgs::msg::Path & plan)
{
  auto inversion_it = findFirstPathInversion(plan);
  if (inversion_it != plan.poses.end()) {
    plan.poses.erase(inversion_it + 1, plan.poses.end());
  }
}

bool PathHandler::checkAndAdvanceToNextInversionSegment(
  const nav_msgs::msg::Path * transformed_plan,
  double lookahead_dist)
{
  // Prune global plan to remove poses up to the first inversion
  removePosesAfterFirstInversion(global_plan_up_to_inversion_);

  // Check if robot has reached the inversion point and should advance to next segment
  if (!global_plan_up_to_inversion_.poses.empty() && !global_plan_.poses.empty()) {
    bool should_advance = false;

    // Check: Robot passed the last path point (sign change or became zero)
    // This indicates the robot has reached the end point and should advance
    if (transformed_plan != nullptr && !transformed_plan->poses.empty()) {
      double last_point_x = transformed_plan->poses.back().pose.position.x;

      // Check if we have a previous value and if sign changed or became zero
      if (!std::isnan(prev_last_point_x_)) {
        bool sign_changed = (prev_last_point_x_ > 0 && last_point_x <= 0) ||
                           (prev_last_point_x_ < 0 && last_point_x >= 0);
        bool became_zero = (last_point_x == 0.0);

        // Check if all points in the remaining path are within lookahead distance
        // This ensures the robot has progressed along the path and prevents false advancement
        // on self-crossing paths or when an earlier part of the path is close to the endpoint
        bool all_points_close = true;
        for (const auto& pose : transformed_plan->poses) {
          double dist = std::hypot(pose.pose.position.x, pose.pose.position.y);
          if (dist > lookahead_dist) {
            all_points_close = false;
            break;
          }
        }

        // Only advance if both sign changed AND all remaining path points are close
        if ((sign_changed || became_zero) && all_points_close) {
          should_advance = true;
          RCLCPP_INFO(logger_,
            "Advancing segment: last_point_x %.3f -> %.3f, all points within %.3f (sign_changed=%d, became_zero=%d)",
            prev_last_point_x_, last_point_x, lookahead_dist, sign_changed, became_zero);
        }
      }

      // Update previous value for next iteration
      prev_last_point_x_ = last_point_x;
    }

    if (should_advance) {
      // Robot has reached inversion point, advance to next segment
      size_t next_segment_start = current_segment_start_idx_ + current_segment_length_;

      if (next_segment_start < global_plan_.poses.size()) {
        // Update tracking for new segment
        current_segment_start_idx_ = next_segment_start;

        // Create new path starting from after the inversion
        global_plan_up_to_inversion_.poses.clear();
        for (size_t i = next_segment_start; i < global_plan_.poses.size(); ++i) {
          global_plan_up_to_inversion_.poses.push_back(global_plan_.poses[i]);
        }

        // Remove poses after the next inversion
        removePosesAfterFirstInversion(global_plan_up_to_inversion_);

        // Store new segment length
        current_segment_length_ = global_plan_up_to_inversion_.poses.size();

        // Reset tracking for new segment
        prev_last_point_x_ = std::numeric_limits<double>::quiet_NaN();

        RCLCPP_INFO(logger_, "Advanced to segment starting at index %zu with %zu poses",
          current_segment_start_idx_, current_segment_length_);

        return true;
      }
    }
  }

  return false;
}

}  // namespace nav2_regulated_pure_pursuit_controller
