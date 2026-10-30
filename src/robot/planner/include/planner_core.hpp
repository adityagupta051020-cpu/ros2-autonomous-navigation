#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot
{

// A grid cell, (x, y) = (column, row)
struct CellIndex {
  int x;
  int y;

  CellIndex(int xx, int yy) : x(xx), y(yy) {}
  CellIndex() : x(0), y(0) {}

  bool operator==(const CellIndex& other) const { return x == other.x && y == other.y; }
  bool operator!=(const CellIndex& other) const { return !(*this == other); }
};

// Entry in the A* open set
struct AStarNode {
  CellIndex index;
  double f_score;  // g + h

  AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};

// Makes std::priority_queue a min-heap on f_score
struct CompareF {
  bool operator()(const AStarNode& a, const AStarNode& b) const {
    return a.f_score > b.f_score;
  }
};

class PlannerCore {
  public:
    explicit PlannerCore(const rclcpp::Logger& logger);

    // A* from start to goal on the map. Returns cells from start to goal
    // inclusive, or an empty vector if there is no path.
    // Cells with cost > obstacle_threshold are walls; cost_weight scales how
    // much lower-cost cells are preferred. Unknown cells (-1) count as free.
    std::vector<CellIndex> planPath(const nav_msgs::msg::OccupancyGrid& map,
                                    const CellIndex& start, const CellIndex& goal,
                                    int obstacle_threshold, double cost_weight) const;

  private:
    rclcpp::Logger logger_;
};

}

#endif
