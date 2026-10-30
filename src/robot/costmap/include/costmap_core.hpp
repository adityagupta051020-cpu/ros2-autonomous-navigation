#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot
{

class CostmapCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger& logger);

    // Set grid geometry and inflation settings. Call once before updateCostmap().
    void initCostmap(double resolution, int width, int height,
                     double origin_x, double origin_y,
                     double inflation_radius, int max_cost);

    // Rebuild the grid from scratch using one laser scan
    void updateCostmap(const sensor_msgs::msg::LaserScan::SharedPtr scan);

    // Latest grid (header is left for the caller to fill in)
    nav_msgs::msg::OccupancyGrid::SharedPtr getCostmapData() const;

  private:
    // Polar (range, angle) in the lidar frame -> grid cell. Returns false if off the grid.
    bool polarToGrid(double range, double angle, int& x_grid, int& y_grid) const;

    // Spread cost around an obstacle cell, falling off linearly with distance
    void inflateObstacle(int x_grid, int y_grid);

    rclcpp::Logger logger_;
    nav_msgs::msg::OccupancyGrid::SharedPtr costmap_data_;

    double inflation_radius_;
    int inflation_cells_;
    int max_cost_;
};

}

#endif
