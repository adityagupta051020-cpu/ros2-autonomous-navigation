#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "planner_core.hpp"

class PlannerNode : public rclcpp::Node {
  public:
    PlannerNode();

  private:
    enum class State { WAITING_FOR_GOAL, WAITING_FOR_ROBOT_TO_REACH_GOAL };

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    // Runs every timer_period_ms: checks for goal reached or timeout
    void timerCallback();

    // Run A* from the robot to the goal and publish the result
    void planPath();
    // Publish an empty path, which tells the controller to stop
    void publishEmptyPath();
    // Sphere at the goal on /goal_marker, coloured by status, plus a text label
    // ("Goal N: <status>") above it, so both show in Foxglove
    void publishGoalMarker(float r, float g, float b, const std::string& status);
    bool worldToGrid(double x, double y, robot::CellIndex& cell) const;

    robot::PlannerCore planner_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    State state_ = State::WAITING_FOR_GOAL;
    nav_msgs::msg::OccupancyGrid::SharedPtr map_;
    geometry_msgs::msg::PointStamped goal_;
    double robot_x_ = 0.0;
    double robot_y_ = 0.0;
    bool have_odom_ = false;
    rclcpp::Time goal_start_time_;
    int goal_count_ = 0;  // numbers the goals in the on-screen label

    double goal_tolerance_;
    int obstacle_threshold_;
    double cost_weight_;
    double goal_timeout_s_;
};

#endif
