/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Locus Robotics
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include "dwb_critics/rotate_to_goal.hpp"
#include <string>
#include <vector>
#include "nav_2d_utils/parameters.hpp"
#include "dwb_core/exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "dwb_core/trajectory_utils.hpp"
#include "angles/angles.h"

PLUGINLIB_EXPORT_CLASS(dwb_critics::RotateToGoalCritic, dwb_core::TrajectoryCritic)

namespace dwb_critics
{



RotateToGoalCritic::CommandTrend::CommandTrend()
{
  reset();
}

void RotateToGoalCritic::CommandTrend::reset()
{
  sign_ = Sign::ZERO;
  positive_only_ = false;
  negative_only_ = false;
  velocity_ = 0.0;
}

bool RotateToGoalCritic::CommandTrend::update(double velocity)
{
  bool flag_set = false;
  if (velocity < 0.0) {
    if (sign_ == Sign::POSITIVE) {
      negative_only_ = true;
      flag_set = true;
    }
    sign_ = Sign::NEGATIVE;
  } else if (velocity > 0.0) {
    if (sign_ == Sign::NEGATIVE) {
      positive_only_ = true;
      flag_set = true;
    }
    sign_ = Sign::POSITIVE;
  }
  velocity_ = velocity;
  return flag_set;
}

bool RotateToGoalCritic::CommandTrend::isOscillating(double velocity)
{
  return (positive_only_ && velocity < 0.0) || (negative_only_ && velocity > 0.0);
}

int RotateToGoalCritic::CommandTrend::signWillFlip(double velocity)
{
  return signbit(velocity) != signbit(velocity_);
}

void RotateToGoalCritic::debrief(const nav_2d_msgs::msg::Twist2D & cmd_vel)
{
  flag_set_ = theta_trend_.update(cmd_vel.theta);
}

inline double hypot_sq(double dx, double dy)
{
  return dx * dx + dy * dy;
}

void RotateToGoalCritic::onInit()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  xy_goal_tolerance_ = nav_2d_utils::searchAndGetParam(
    node,
    dwb_plugin_name_ + ".xy_goal_tolerance", 0.25);
  xy_goal_tolerance_sq_ = xy_goal_tolerance_ * xy_goal_tolerance_;
  double stopped_xy_velocity = nav_2d_utils::searchAndGetParam(
    node,
    dwb_plugin_name_ + ".trans_stopped_velocity", 0.25);
  stopped_xy_velocity_sq_ = stopped_xy_velocity * stopped_xy_velocity;
  slowing_factor_ = nav_2d_utils::searchAndGetParam(
    node,
    dwb_plugin_name_ + "." + name_ + ".slowing_factor", 5.0);
  lookahead_time_ = nav_2d_utils::searchAndGetParam(
    node,
    dwb_plugin_name_ + "." + name_ + ".lookahead_time", -1.0);
  reset();
}

void RotateToGoalCritic::reset()
{
  in_window_ = false;
  rotating_ = false;
  theta_trend_.reset();
}

bool RotateToGoalCritic::prepare(
  const geometry_msgs::msg::Pose2D & pose, const nav_2d_msgs::msg::Twist2D & vel,
  const geometry_msgs::msg::Pose2D & goal,
  const nav_2d_msgs::msg::Path2D &)
{
  double dxy_sq = hypot_sq(pose.x - goal.x, pose.y - goal.y);
  in_window_ = in_window_ || dxy_sq <= xy_goal_tolerance_sq_;
  current_xy_speed_sq_ = hypot_sq(vel.x, vel.y);
  rotating_ = rotating_ || (in_window_ && current_xy_speed_sq_ <= stopped_xy_velocity_sq_);
  goal_yaw_ = goal.theta;
  return true;
}

double RotateToGoalCritic::scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj)
{
  // If we're not sufficiently close to the goal, we don't care what the twist is
  if (!in_window_) {
    return 0.0;
  } else if (!rotating_) {
    double speed_sq = hypot_sq(traj.velocity.x, traj.velocity.y);
    if (speed_sq >= current_xy_speed_sq_) {
      throw dwb_core::IllegalTrajectoryException(name_, "Not slowing down near goal.");
    }
    return speed_sq * slowing_factor_ + scoreRotation(traj);
  }

  // If we're sufficiently close to the goal, any transforming velocity is invalid
  if (fabs(traj.velocity.x) > 0 || fabs(traj.velocity.y) > 0) {
    throw dwb_core::
          IllegalTrajectoryException(name_, "Nonrotation command near goal.");
  }

  return scoreRotation(traj);
}

double RotateToGoalCritic::scoreRotation(const dwb_msgs::msg::Trajectory2D & traj)
{
  if (traj.poses.empty()) {
    throw dwb_core::IllegalTrajectoryException(name_, "Empty trajectory.");
  }

  double end_yaw;
  if (lookahead_time_ >= 0.0) {
    geometry_msgs::msg::Pose2D eval_pose = dwb_core::projectPose(traj, lookahead_time_);
    end_yaw = eval_pose.theta;
  } else {
    end_yaw = traj.poses.back().theta;
  }
  return fabs(angles::shortest_angular_distance(end_yaw, goal_yaw_)) ;
         //10.0*theta_trend_.signWillFlip(traj.velocity.theta); 
}

}  // namespace dwb_critics
