#ifndef OCCUPANCY_GRID_VALUE_NODE_HPP
#define OCCUPANCY_GRID_VALUE_NODE_HPP

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

class OccupancyGridValueNode : public rclcpp::Node
{
public:
    OccupancyGridValueNode();

private:
    void pointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void gridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub_;

    nav_msgs::msg::OccupancyGrid::SharedPtr occupancy_grid_;
};

#endif // OCCUPANCY_GRID_VALUE_NODE_HPP
