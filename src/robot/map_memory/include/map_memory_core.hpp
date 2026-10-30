#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot
{

class MapMemoryCore {
  public:
    explicit MapMemoryCore(const rclcpp::Logger& logger);

    // Set global map geometry. Call once before fuseCostmap().
    void initMap(double resolution, int width, int height,
                 double origin_x, double origin_y, const std::string& frame_id);

    // Merge a robot-relative costmap into the global map, given the robot's
    // world pose (x, y, yaw) at the time the costmap was taken
    void fuseCostmap(const nav_msgs::msg::OccupancyGrid& costmap,
                     double robot_x, double robot_y, double robot_yaw);

    nav_msgs::msg::OccupancyGrid::SharedPtr getMap() const;

  private:
    rclcpp::Logger logger_;
    nav_msgs::msg::OccupancyGrid::SharedPtr global_map_;
};

}

#endif
