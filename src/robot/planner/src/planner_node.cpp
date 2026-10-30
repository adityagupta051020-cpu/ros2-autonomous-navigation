#include <chrono>
#include <cmath>
#include <memory>

#include "planner_node.hpp"

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {
  // Defaults are used if params.yaml doesn't set a value
  goal_tolerance_ = this->declare_parameter<double>("goal_tolerance", 0.5);
  obstacle_threshold_ = this->declare_parameter<int>("obstacle_threshold", 50);
  cost_weight_ = this->declare_parameter<double>("cost_weight", 2.0);
  goal_timeout_s_ = this->declare_parameter<double>("goal_timeout_s", 120.0);
  const int timer_period_ms = this->declare_parameter<int>("timer_period_ms", 500);

  // transient_local matches map_memory's publisher, so we get the last map even
  // if it was published before this node started
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", rclcpp::QoS(1).transient_local(),
    std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
  // transient_local so Foxglove still shows the marker after a reload. Depth 2
  // keeps both the latest sphere and the latest label.
  goal_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
    "/goal_marker", rclcpp::QoS(2).transient_local());

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(timer_period_ms), std::bind(&PlannerNode::timerCallback, this));
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  map_ = msg;
  // The map grows as the robot explores, so the old path may now cross a wall
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    planPath();
  }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  if (map_ && !msg->header.frame_id.empty() && msg->header.frame_id != map_->header.frame_id) {
    RCLCPP_WARN(this->get_logger(), "Goal is in frame '%s' but the map is in '%s'; treating it as map frame",
      msg->header.frame_id.c_str(), map_->header.frame_id.c_str());
  }

  goal_ = *msg;
  goal_start_time_ = this->now();
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  ++goal_count_;
  RCLCPP_INFO(this->get_logger(), "New goal: (%.2f, %.2f)", goal_.point.x, goal_.point.y);
  publishGoalMarker(1.0, 0.55, 0.0, "driving");  // orange
  planPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;
  have_odom_ = true;
}

void PlannerNode::timerCallback() {
  if (state_ != State::WAITING_FOR_ROBOT_TO_REACH_GOAL || !have_odom_) {
    return;
  }

  const double distance = std::hypot(goal_.point.x - robot_x_, goal_.point.y - robot_y_);
  if (distance < goal_tolerance_) {
    RCLCPP_INFO(this->get_logger(), "Goal reached");
    state_ = State::WAITING_FOR_GOAL;
    publishEmptyPath();
    publishGoalMarker(0.1, 0.85, 0.2, "reached");  // green
  } else if ((this->now() - goal_start_time_).seconds() > goal_timeout_s_) {
    RCLCPP_WARN(this->get_logger(), "Goal timed out after %.0f s, giving up", goal_timeout_s_);
    state_ = State::WAITING_FOR_GOAL;
    publishEmptyPath();
    publishGoalMarker(0.9, 0.1, 0.1, "timed out");  // red
  }
}

void PlannerNode::planPath() {
  if (!map_ || !have_odom_) {
    RCLCPP_WARN(this->get_logger(), "Cannot plan yet: %s", !map_ ? "no map" : "no odometry");
    return;
  }

  robot::CellIndex start, goal;
  if (!worldToGrid(robot_x_, robot_y_, start) || !worldToGrid(goal_.point.x, goal_.point.y, goal)) {
    RCLCPP_WARN(this->get_logger(), "Robot or goal is outside the map");
    publishEmptyPath();
    publishGoalMarker(0.9, 0.1, 0.1, "outside the map");  // red
    return;
  }

  const auto cells = planner_.planPath(*map_, start, goal, obstacle_threshold_, cost_weight_);
  if (cells.empty()) {
    publishEmptyPath();
    publishGoalMarker(0.9, 0.1, 0.1, "no safe route");  // red
    return;
  }

  nav_msgs::msg::Path path;
  path.header.stamp = this->now();
  path.header.frame_id = map_->header.frame_id;

  const auto& info = map_->info;
  for (const auto& cell : cells) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    // Waypoints are cell centres
    pose.pose.position.x = info.origin.position.x + (cell.x + 0.5) * info.resolution;
    pose.pose.position.y = info.origin.position.y + (cell.y + 0.5) * info.resolution;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  // End exactly on the clicked point rather than its cell centre
  path.poses.back().pose.position.x = goal_.point.x;
  path.poses.back().pose.position.y = goal_.point.y;

  path_pub_->publish(path);
  RCLCPP_INFO(this->get_logger(), "Published path with %zu waypoints", path.poses.size());
}

void PlannerNode::publishEmptyPath() {
  nav_msgs::msg::Path path;
  path.header.stamp = this->now();
  path.header.frame_id = map_ ? map_->header.frame_id : "sim_world";
  path_pub_->publish(path);
}

void PlannerNode::publishGoalMarker(float r, float g, float b, const std::string& status) {
  visualization_msgs::msg::Marker marker;
  marker.header.stamp = this->now();
  marker.header.frame_id = map_ ? map_->header.frame_id : "sim_world";
  marker.ns = "goal";
  marker.id = 0;  // same id every time, so each goal replaces the last marker
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = goal_.point.x;
  marker.pose.position.y = goal_.point.y;
  marker.pose.position.z = 0.4;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = marker.scale.y = marker.scale.z = 0.8;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 0.9;
  goal_marker_pub_->publish(marker);

  // Text label floating above the sphere, e.g. "Goal 2: reached"
  visualization_msgs::msg::Marker label = marker;
  label.id = 1;
  label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  label.text = "Goal " + std::to_string(goal_count_) + ": " + status;
  label.pose.position.z = 2.0;
  label.scale.x = label.scale.y = 0.0;  // only scale.z (text height) is used
  label.scale.z = 0.7;
  label.color.r = label.color.g = label.color.b = 1.0;  // white text
  label.color.a = 1.0;
  goal_marker_pub_->publish(label);
}

bool PlannerNode::worldToGrid(double x, double y, robot::CellIndex& cell) const {
  const auto& info = map_->info;
  cell.x = static_cast<int>(std::floor((x - info.origin.position.x) / info.resolution));
  cell.y = static_cast<int>(std::floor((y - info.origin.position.y) / info.resolution));
  return cell.x >= 0 && cell.x < static_cast<int>(info.width) &&
         cell.y >= 0 && cell.y < static_cast<int>(info.height);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
