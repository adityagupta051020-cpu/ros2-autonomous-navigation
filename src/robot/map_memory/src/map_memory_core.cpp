#include <algorithm>
#include <cmath>

#include "map_memory_core.hpp"

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger)
  : logger_(logger),
    global_map_(std::make_shared<nav_msgs::msg::OccupancyGrid>()) {}

void MapMemoryCore::initMap(double resolution, int width, int height,
                            double origin_x, double origin_y, const std::string& frame_id) {
  global_map_->header.frame_id = frame_id;
  global_map_->info.resolution = resolution;
  global_map_->info.width = width;
  global_map_->info.height = height;
  global_map_->info.origin.position.x = origin_x;
  global_map_->info.origin.position.y = origin_y;
  global_map_->info.origin.orientation.w = 1.0;
  // Nothing seen yet: start fully free
  global_map_->data.assign(width * height, 0);

  RCLCPP_INFO(logger_, "Global map initialized: %dx%d cells at %.2f m in '%s'",
    width, height, resolution, frame_id.c_str());
}

void MapMemoryCore::fuseCostmap(const nav_msgs::msg::OccupancyGrid& costmap,
                                double robot_x, double robot_y, double robot_yaw) {
  const auto& map_info = global_map_->info;
  const auto& cm_info = costmap.info;
  const int map_width = static_cast<int>(map_info.width);
  const int map_height = static_cast<int>(map_info.height);
  const int cm_width = static_cast<int>(cm_info.width);
  const int cm_height = static_cast<int>(cm_info.height);

  const double cos_yaw = std::cos(robot_yaw);
  const double sin_yaw = std::sin(robot_yaw);

  // Loop over global cells and look each one up in the costmap (world -> robot),
  // rather than pushing costmap cells out (robot -> world). Going this way round
  // means every global cell gets a value, so rotated costmaps leave no gaps.
  for (int my = 0; my < map_height; ++my) {
    for (int mx = 0; mx < map_width; ++mx) {
      // Centre of this global cell in world coordinates
      const double x_w = map_info.origin.position.x + (mx + 0.5) * map_info.resolution;
      const double y_w = map_info.origin.position.y + (my + 0.5) * map_info.resolution;

      // Undo the robot's translation, then its rotation (inverse of robot -> world)
      const double dx = x_w - robot_x;
      const double dy = y_w - robot_y;
      const double x_r =  cos_yaw * dx + sin_yaw * dy;
      const double y_r = -sin_yaw * dx + cos_yaw * dy;

      const int cx = static_cast<int>(std::floor((x_r - cm_info.origin.position.x) / cm_info.resolution));
      const int cy = static_cast<int>(std::floor((y_r - cm_info.origin.position.y) / cm_info.resolution));
      if (cx < 0 || cx >= cm_width || cy < 0 || cy >= cm_height) {
        continue;  // outside the costmap's footprint: keep what we knew
      }

      const int8_t cost = costmap.data[cy * cm_width + cx];
      if (cost < 0) {
        continue;  // unknown in the costmap: keep what we knew
      }

      // The costmap marks unseen space as free, so a plain overwrite would erase
      // obstacles that are now hidden behind something. In a static world it's
      // safe to only ever raise a cell's cost.
      int8_t& cell = global_map_->data[my * map_width + mx];
      cell = std::max(cell, cost);
    }
  }
}

nav_msgs::msg::OccupancyGrid::SharedPtr MapMemoryCore::getMap() const {
  return global_map_;
}

}
