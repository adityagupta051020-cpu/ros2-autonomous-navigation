#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "control_core.hpp"

class ControlNode : public rclcpp::Node {
  public:
    ControlNode();

  private:
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    // Runs every control_period_ms
    void controlLoop();
    // Publish a single zero command and stop following
    void stop();

    robot::ControlCore control_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    nav_msgs::msg::Path::SharedPtr path_;
    double robot_x_ = 0.0;
    double robot_y_ = 0.0;
    double robot_yaw_ = 0.0;
    bool have_odom_ = false;
    // Only publish /cmd_vel while following a path, so Teleop still works otherwise
    bool following_ = false;
};

#endif
