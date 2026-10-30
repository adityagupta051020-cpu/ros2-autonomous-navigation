#include <chrono>
#include <cmath>
#include <memory>

#include "control_node.hpp"

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore(this->get_logger())) {
  // Defaults are used if params.yaml doesn't set a value
  robot::ControlParams params;
  params.lookahead_distance = this->declare_parameter<double>("lookahead_distance", 1.5);
  params.linear_speed = this->declare_parameter<double>("linear_speed", 0.8);
  params.max_angular_speed = this->declare_parameter<double>("max_angular_speed", 1.0);
  params.rotate_in_place_angle = this->declare_parameter<double>("rotate_in_place_angle", 1.0);
  params.goal_tolerance = this->declare_parameter<double>("goal_tolerance", 0.3);
  const int control_period_ms = this->declare_parameter<int>("control_period_ms", 100);
  control_.setParams(params);

  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/path", 10, std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(control_period_ms), std::bind(&ControlNode::controlLoop, this));
}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
  if (msg->poses.empty()) {
    // The planner sends an empty path when the goal is reached or planning fails
    if (following_) {
      RCLCPP_INFO(this->get_logger(), "Received empty path, stopping");
      stop();
    }
    return;
  }
  path_ = msg;
  // A replanned path starts at the robot, so track progress from its beginning
  control_.resetProgress();
  following_ = true;
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;

  // Quaternion -> yaw (rotation about z), valid for a robot on flat ground
  const auto& q = msg->pose.pose.orientation;
  robot_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  have_odom_ = true;
}

void ControlNode::controlLoop() {
  if (!following_ || !path_ || !have_odom_) {
    return;
  }

  bool reached_goal = false;
  const auto cmd = control_.computeVelocity(*path_, robot_x_, robot_y_, robot_yaw_, reached_goal);
  if (reached_goal) {
    RCLCPP_INFO(this->get_logger(), "Reached end of path");
    stop();
    return;
  }
  cmd_vel_pub_->publish(cmd);
}

void ControlNode::stop() {
  following_ = false;
  path_.reset();
  cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
