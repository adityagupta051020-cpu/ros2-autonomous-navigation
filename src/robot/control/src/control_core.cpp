#include <algorithm>
#include <cmath>

#include "control_core.hpp"

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger& logger)
  : logger_(logger), params_{1.5, 0.8, 1.0, 1.0, 0.3} {}

void ControlCore::setParams(const ControlParams& params) {
  params_ = params;
}

void ControlCore::resetProgress() {
  progress_index_ = 0;
}

size_t ControlCore::findLookaheadIndex(const nav_msgs::msg::Path& path, double robot_x, double robot_y) {
  auto distanceTo = [&](size_t i) {
    const auto& p = path.poses[i].pose.position;
    return std::hypot(p.x - robot_x, p.y - robot_y);
  };

  // Advance to the closest waypoint, starting from where we were last time
  for (size_t i = progress_index_ + 1; i < path.poses.size(); ++i) {
    if (distanceTo(i) < distanceTo(progress_index_)) {
      progress_index_ = i;
    }
  }

  for (size_t i = progress_index_; i < path.poses.size(); ++i) {
    if (distanceTo(i) >= params_.lookahead_distance) {
      return i;
    }
  }
  // Near the end of the path: aim for the last waypoint
  return path.poses.size() - 1;
}

geometry_msgs::msg::Twist ControlCore::computeVelocity(const nav_msgs::msg::Path& path,
                                                       double robot_x, double robot_y, double robot_yaw,
                                                       bool& reached_goal) {
  geometry_msgs::msg::Twist cmd;  // all zeros = stop
  reached_goal = false;
  if (path.poses.empty()) {
    return cmd;
  }
  progress_index_ = std::min(progress_index_, path.poses.size() - 1);

  const auto& goal = path.poses.back().pose.position;
  if (std::hypot(goal.x - robot_x, goal.y - robot_y) < params_.goal_tolerance) {
    reached_goal = true;
    return cmd;
  }

  const auto& target = path.poses[findLookaheadIndex(path, robot_x, robot_y)].pose.position;

  // Target in the robot's frame: x forward, y to the left
  const double dx = target.x - robot_x;
  const double dy = target.y - robot_y;
  const double x_r =  std::cos(robot_yaw) * dx + std::sin(robot_yaw) * dy;
  const double y_r = -std::sin(robot_yaw) * dx + std::cos(robot_yaw) * dy;
  const double heading_error = std::atan2(y_r, x_r);

  if (std::abs(heading_error) > params_.rotate_in_place_angle) {
    // Target is well off to the side or behind: turn on the spot instead of
    // swinging a wide arc that could clip an obstacle
    cmd.angular.z = std::copysign(params_.max_angular_speed, heading_error);
    return cmd;
  }

  // Pure Pursuit: the arc through the robot and the target has curvature 2y / L^2
  const double distance_sq = x_r * x_r + y_r * y_r;
  const double curvature = 2.0 * y_r / distance_sq;

  cmd.linear.x = params_.linear_speed;
  cmd.angular.z = std::clamp(params_.linear_speed * curvature,
                             -params_.max_angular_speed, params_.max_angular_speed);
  return cmd;
}

}
