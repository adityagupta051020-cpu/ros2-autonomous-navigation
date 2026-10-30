#include <algorithm>
#include <cmath>

#include "costmap_core.hpp"

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger)
  : logger_(logger),
    costmap_data_(std::make_shared<nav_msgs::msg::OccupancyGrid>()),
    inflation_radius_(1.0),
    inflation_cells_(0),
    max_cost_(100) {}

void CostmapCore::initCostmap(double resolution, int width, int height,
                              double origin_x, double origin_y,
                              double inflation_radius, int max_cost) {
  costmap_data_->info.resolution = resolution;
  costmap_data_->info.width = width;
  costmap_data_->info.height = height;
  // Origin is the real-world position of cell (0, 0), the bottom-left corner
  costmap_data_->info.origin.position.x = origin_x;
  costmap_data_->info.origin.position.y = origin_y;
  costmap_data_->info.origin.orientation.w = 1.0;
  costmap_data_->data.assign(width * height, 0);

  inflation_radius_ = inflation_radius;
  inflation_cells_ = static_cast<int>(std::ceil(inflation_radius / resolution));
  max_cost_ = max_cost;

  RCLCPP_INFO(logger_, "Costmap initialized: %dx%d cells at %.2f m, inflation %.2f m",
    width, height, resolution, inflation_radius);
}

void CostmapCore::updateCostmap(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  const int width = static_cast<int>(costmap_data_->info.width);
  const int height = static_cast<int>(costmap_data_->info.height);

  // Start every scan from a clean, free grid
  std::fill(costmap_data_->data.begin(), costmap_data_->data.end(), 0);

  // Mark each cell hit by a beam. Several beams can hit the same cell,
  // so track which ones are marked to inflate each cell only once.
  std::vector<bool> is_obstacle(width * height, false);
  std::vector<std::pair<int, int>> obstacles;

  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    const double range = scan->ranges[i];
    if (!std::isfinite(range) || range < scan->range_min || range > scan->range_max) {
      continue;
    }
    const double angle = scan->angle_min + i * scan->angle_increment;

    int x_grid, y_grid;
    if (!polarToGrid(range, angle, x_grid, y_grid)) {
      continue;
    }
    const int index = y_grid * width + x_grid;
    if (!is_obstacle[index]) {
      is_obstacle[index] = true;
      obstacles.emplace_back(x_grid, y_grid);
    }
  }

  for (const auto& [x_grid, y_grid] : obstacles) {
    inflateObstacle(x_grid, y_grid);
  }
}

bool CostmapCore::polarToGrid(double range, double angle, int& x_grid, int& y_grid) const {
  const double x = range * std::cos(angle);
  const double y = range * std::sin(angle);

  const auto& info = costmap_data_->info;
  x_grid = static_cast<int>(std::floor((x - info.origin.position.x) / info.resolution));
  y_grid = static_cast<int>(std::floor((y - info.origin.position.y) / info.resolution));

  return x_grid >= 0 && x_grid < static_cast<int>(info.width) &&
         y_grid >= 0 && y_grid < static_cast<int>(info.height);
}

void CostmapCore::inflateObstacle(int x_grid, int y_grid) {
  const int width = static_cast<int>(costmap_data_->info.width);
  const int height = static_cast<int>(costmap_data_->info.height);
  const double resolution = costmap_data_->info.resolution;

  // Only visit the square of cells that could be within the radius
  for (int dy = -inflation_cells_; dy <= inflation_cells_; ++dy) {
    for (int dx = -inflation_cells_; dx <= inflation_cells_; ++dx) {
      const int nx = x_grid + dx;
      const int ny = y_grid + dy;
      if (nx < 0 || nx >= width || ny < 0 || ny >= height) {
        continue;
      }

      const double distance = std::hypot(dx, dy) * resolution;
      if (distance > inflation_radius_) {
        continue;
      }

      // distance 0 (the obstacle itself) gets max_cost, the radius edge gets 0
      const int cost = static_cast<int>(max_cost_ * (1.0 - distance / inflation_radius_));
      int8_t& cell = costmap_data_->data[ny * width + nx];
      // Only raise a cell's cost, so overlapping obstacles keep the highest value
      cell = static_cast<int8_t>(std::max(static_cast<int>(cell), cost));
    }
  }
}

nav_msgs::msg::OccupancyGrid::SharedPtr CostmapCore::getCostmapData() const {
  return costmap_data_;
}

}
