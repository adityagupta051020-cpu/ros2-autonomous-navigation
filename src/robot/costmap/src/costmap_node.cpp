#include <chrono>
#include <memory>

#include "costmap_node.hpp"

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // Defaults are used if params.yaml doesn't set a value
  const double resolution = this->declare_parameter<double>("resolution", 0.1);
  const int width = this->declare_parameter<int>("width", 300);
  const int height = this->declare_parameter<int>("height", 300);
  const double origin_x = this->declare_parameter<double>("origin_x", -15.0);
  const double origin_y = this->declare_parameter<double>("origin_y", -15.0);
  const double inflation_radius = this->declare_parameter<double>("inflation_radius", 1.0);
  const int max_cost = this->declare_parameter<int>("max_cost", 100);

  costmap_.initCostmap(resolution, width, height, origin_x, origin_y, inflation_radius, max_cost);

  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10, std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  costmap_.updateCostmap(msg);

  auto costmap = costmap_.getCostmapData();
  // Same stamp and frame as the scan, so the grid is drawn attached to the lidar
  costmap->header = msg->header;
  costmap_pub_->publish(*costmap);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
