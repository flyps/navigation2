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

#ifndef NAV2_REGULATED_PURE_PURSUIT_CONTROLLER__PATH_HANDLER_HPP_
#define NAV2_REGULATED_PURE_PURSUIT_CONTROLLER__PATH_HANDLER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_core/controller_exceptions.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"

namespace nav2_regulated_pure_pursuit_controller
{

/**
 * @class nav2_regulated_pure_pursuit_controller::PathHandler
 * @brief Handles input paths to transform them to local frames required
 */
class PathHandler
{
public:
  /**
   * @brief Constructor for nav2_regulated_pure_pursuit_controller::PathHandler
   */
  PathHandler(
    tf2::Duration transform_tolerance,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros);

  /**
   * @brief Destrructor for nav2_regulated_pure_pursuit_controller::PathHandler
   */
  ~PathHandler() = default;

  /**
   * @brief Transforms global plan into same frame as pose and clips poses ineligible for lookaheadPoint
   * Points ineligible to be selected as a lookahead point if they are any of the following:
   * - Outside the local_costmap (collision avoidance cannot be assured)
   * @param pose pose to transform
   * @param max_robot_pose_search_dist Distance to search for matching nearest path point
   * @param reject_unit_path If true, fail if path has only one pose
   * @param prune_plan If true, prune the global plan after transformation
   * @return Path in new frame
   */
  nav_msgs::msg::Path transformGlobalPlan(
    const geometry_msgs::msg::PoseStamped & pose,
    double max_robot_pose_search_dist, bool reject_unit_path = false,
    bool prune_plan = true);

  /**
   * @brief Transform a pose to another frame.
   * @param frame Frame ID to transform to
   * @param in_pose Pose input to transform
   * @param out_pose transformed output
   * @return bool if successful
   */
  bool transformPose(
    const std::string frame,
    const geometry_msgs::msg::PoseStamped & in_pose,
    geometry_msgs::msg::PoseStamped & out_pose) const;

  void setPlan(const nav_msgs::msg::Path & path)
  {
    global_plan_ = path;
    global_plan_up_to_inversion_ = path;
    current_segment_start_idx_ = 0;
    removePosesAfterFirstInversion(global_plan_up_to_inversion_);
    current_segment_length_ = global_plan_up_to_inversion_.poses.size();
  }

  nav_msgs::msg::Path getPlan() {return global_plan_;}

  /**
   * @brief Set inversion tolerance parameters
   * @param xy_tolerance XY distance tolerance in meters
   * @param yaw_tolerance Yaw angle tolerance in radians
   */
  void setInversionTolerances(double xy_tolerance, double yaw_tolerance)
  {
    inversion_xy_tolerance_ = xy_tolerance;
    inversion_yaw_tolerance_ = yaw_tolerance;
  }

  /**
   * @brief Check if robot reached inversion point and advance to next path segment
   * This manages the global_plan_up_to_inversion_ by pruning at inversions and
   * advancing to the next segment when the robot reaches an inversion point.
   * @param robot_pose Current robot pose in map frame
   * @return true if a new path segment was activated
   */
  bool checkAndAdvanceToNextInversionSegment(const geometry_msgs::msg::PoseStamped & robot_pose);

protected:
  /**
   * @brief Calculates the shortest angular distance between two angles
   * @param from Starting angle in radians
   * @param to Target angle in radians
   * @return Shortest angular distance in radians
   */
  static inline double shortest_angular_distance(double from, double to)
  {
    double delta = to - from;
    // Normalize to [-pi, pi]
    while (delta > M_PI) {delta -= 2.0 * M_PI;}
    while (delta < -M_PI) {delta += 2.0 * M_PI;}
    return delta;
  }

  /**
   * @brief Find the first path inversion (cusp) point
   * @param plan Path to search for inversion
   * @return Iterator to the first inversion point, or end() if none found
   */
  static nav_msgs::msg::Path::_poses_type::iterator findFirstPathInversion(
    nav_msgs::msg::Path & plan);

  /**
   * @brief Remove poses after the first inversion in the path
   * @param plan Path to prune
   */
  static void removePosesAfterFirstInversion(nav_msgs::msg::Path & plan);

  /**
   * @brief Check if robot is within tolerance of inversion point
   * @param robot_pose Current robot pose
   * @param inversion_pose Inversion point pose
   * @return true if robot is within tolerances
   */
  bool isWithinInversionTolerances(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::PoseStamped & inversion_pose);
  /**
   * Get the greatest extent of the costmap in meters from the center.
   * @return max of distance from center in meters to edge of costmap
   */
  double getCostmapMaxExtent() const;

  rclcpp::Logger logger_ {rclcpp::get_logger("RPPPathHandler")};
  tf2::Duration transform_tolerance_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav_msgs::msg::Path global_plan_;
  nav_msgs::msg::Path global_plan_up_to_inversion_;
  double inversion_xy_tolerance_{0.2};
  double inversion_yaw_tolerance_{0.4};
  size_t current_segment_start_idx_{0};
  size_t current_segment_length_{0};
};

}  // namespace nav2_regulated_pure_pursuit_controller

#endif  // NAV2_REGULATED_PURE_PURSUIT_CONTROLLER__PATH_HANDLER_HPP_
