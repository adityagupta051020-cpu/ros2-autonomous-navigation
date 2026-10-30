#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace robot
{

struct ControlParams {
  double lookahead_distance;
  double linear_speed;
  double max_angular_speed;
  double rotate_in_place_angle;
  double goal_tolerance;
};

class ControlCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    ControlCore(const rclcpp::Logger& logger);

    void setParams(const ControlParams& params);

    // Forget progress along the old path (call when a new path arrives)
    void resetProgress();

    // Pure Pursuit: velocity command to follow the path from the given pose.
    // Sets reached_goal when within goal_tolerance of the last waypoint.
    geometry_msgs::msg::Twist computeVelocity(const nav_msgs::msg::Path& path,
                                              double robot_x, double robot_y, double robot_yaw,
                                              bool& reached_goal);

  private:
    // Index of the first waypoint at least lookahead_distance from the robot,
    // searching forward from the closest waypoint so the robot never steers backwards
    size_t findLookaheadIndex(const nav_msgs::msg::Path& path, double robot_x, double robot_y);

    rclcpp::Logger logger_;
    ControlParams params_;
    // Closest waypoint so far; only moves forward along the path
    size_t progress_index_ = 0;
};

}

#endif
