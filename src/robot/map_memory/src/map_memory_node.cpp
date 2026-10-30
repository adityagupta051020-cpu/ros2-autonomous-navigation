#include <chrono>
#include <cmath>
#include <memory>

#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
  // Defaults are used if params.yaml doesn't set a value
  const double resolution = this->declare_parameter<double>("resolution", 0.1);
  const int width = this->declare_parameter<int>("width", 300);
  const int height = this->declare_parameter<int>("height", 300);
  const double origin_x = this->declare_parameter<double>("origin_x", -15.0);
  const double origin_y = this->declare_parameter<double>("origin_y", -15.0);
  const std::string frame_id = this->declare_parameter<std::string>("frame_id", "sim_world");
  distance_threshold_ = this->declare_parameter<double>("distance_threshold", 1.5);
  const int update_period_ms = this->declare_parameter<int>("update_period_ms", 1000);

  map_memory_.initMap(resolution, width, height, origin_x, origin_y, frame_id);

  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

  // transient_local keeps the last map for late subscribers, since /map is only
  // published when it changes (a planner started later still receives it)
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "/map", rclcpp::QoS(1).transient_local());

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(update_period_ms), std::bind(&MapMemoryNode::updateMap, this));
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  latest_costmap_ = msg;
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;

  // Quaternion -> yaw (rotation about z), valid for a robot on flat ground
  const auto& q = msg->pose.pose.orientation;
  robot_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  have_odom_ = true;

  if (std::hypot(robot_x_ - last_x_, robot_y_ - last_y_) >= distance_threshold_) {
    should_update_ = true;
  }
}

void MapMemoryNode::updateMap() {
  if (!should_update_ || !latest_costmap_ || !have_odom_) {
    return;
  }

  map_memory_.fuseCostmap(*latest_costmap_, robot_x_, robot_y_, robot_yaw_);

  auto map = map_memory_.getMap();
  map->header.stamp = this->now();
  map_pub_->publish(*map);

  last_x_ = robot_x_;
  last_y_ = robot_y_;
  should_update_ = false;
  RCLCPP_INFO(this->get_logger(), "Fused costmap at (%.2f, %.2f), yaw %.2f rad", robot_x_, robot_y_, robot_yaw_);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
