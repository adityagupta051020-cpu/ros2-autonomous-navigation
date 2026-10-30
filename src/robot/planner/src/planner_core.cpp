#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

#include "planner_core.hpp"

namespace robot
{

PlannerCore::PlannerCore(const rclcpp::Logger& logger)
: logger_(logger) {}

std::vector<CellIndex> PlannerCore::planPath(const nav_msgs::msg::OccupancyGrid& map,
                                             const CellIndex& start, const CellIndex& goal,
                                             int obstacle_threshold, double cost_weight) const {
  const int width = static_cast<int>(map.info.width);
  const int height = static_cast<int>(map.info.height);

  auto inBounds = [&](const CellIndex& c) {
    return c.x >= 0 && c.x < width && c.y >= 0 && c.y < height;
  };
  auto toIndex = [&](const CellIndex& c) { return c.y * width + c.x; };
  // Unknown (-1) is treated as free, so the robot will plan into unexplored space
  auto cellCost = [&](const CellIndex& c) { return std::max<int>(0, map.data[toIndex(c)]); };
  auto heuristic = [&](const CellIndex& c) { return std::hypot(c.x - goal.x, c.y - goal.y); };

  if (!inBounds(start) || !inBounds(goal)) {
    RCLCPP_WARN(logger_, "Start or goal is outside the map");
    return {};
  }
  // The start isn't checked: the robot may legitimately be inside an inflated zone
  if (cellCost(goal) > obstacle_threshold) {
    RCLCPP_WARN(logger_, "Goal (%d, %d) is inside an obstacle (cost %d)", goal.x, goal.y, cellCost(goal));
    return {};
  }

  const int num_cells = width * height;
  std::vector<double> g_score(num_cells, std::numeric_limits<double>::infinity());
  std::vector<int> came_from(num_cells, -1);
  std::vector<bool> closed(num_cells, false);

  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;
  g_score[toIndex(start)] = 0.0;
  open_set.emplace(start, heuristic(start));

  // 8-connected: straight moves cost 1 cell, diagonals sqrt(2)
  static const int dx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  static const int dy[8] = {0, 0, 1, -1, 1, -1, 1, -1};

  while (!open_set.empty()) {
    const CellIndex current = open_set.top().index;
    open_set.pop();

    const int current_index = toIndex(current);
    // A cell can be queued several times with different f-scores; only expand it once
    if (closed[current_index]) {
      continue;
    }
    closed[current_index] = true;

    if (current == goal) {
      std::vector<CellIndex> path;
      for (int i = current_index; i != -1; i = came_from[i]) {
        path.emplace_back(i % width, i / width);
      }
      std::reverse(path.begin(), path.end());
      return path;
    }

    for (int k = 0; k < 8; ++k) {
      const CellIndex neighbor(current.x + dx[k], current.y + dy[k]);
      if (!inBounds(neighbor)) {
        continue;
      }
      const int neighbor_index = toIndex(neighbor);
      if (closed[neighbor_index]) {
        continue;
      }
      // Cells above the threshold are walls, except when escaping: if the robot
      // starts close to an obstacle, every nearby cell is above the threshold, so
      // allow moves from a high-cost cell to an equal-or-lower one (never into an
      // actual obstacle at 100). The path then climbs out "downhill".
      const int current_cost = cellCost(current);
      const int neighbor_cost = cellCost(neighbor);
      const bool escaping = current_cost > obstacle_threshold &&
                            neighbor_cost <= current_cost && neighbor_cost < 100;
      if (neighbor_cost > obstacle_threshold && !escaping) {
        continue;
      }

      // Distance, scaled up for cells closer to obstacles
      const double step = (k < 4) ? 1.0 : M_SQRT2;
      const double penalty = 1.0 + cost_weight * neighbor_cost / 100.0;
      const double tentative_g = g_score[current_index] + step * penalty;

      if (tentative_g < g_score[neighbor_index]) {
        g_score[neighbor_index] = tentative_g;
        came_from[neighbor_index] = current_index;
        open_set.emplace(neighbor, tentative_g + heuristic(neighbor));
      }
    }
  }

  RCLCPP_WARN(logger_, "No path found from (%d, %d) to (%d, %d)", start.x, start.y, goal.x, goal.y);
  return {};
}

}
