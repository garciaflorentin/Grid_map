#ifndef OCCUPANCYGRIDFUSION_HPP_
#define OCCUPANCYGRIDFUSION_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

class OccupancyGridFusion : public rclcpp::Node
{
public:
    OccupancyGridFusion();

private:
    void grid1Callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void grid2Callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void fuseGrids();
    int mapValue(int value, int min_in, int max_in, int min_out, int max_out);

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid1_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid2_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr fused_grid_pub_;

    nav_msgs::msg::OccupancyGrid::SharedPtr grid1_;
    nav_msgs::msg::OccupancyGrid::SharedPtr grid2_;
    double weight1_;
    double weight2_;
};

#endif // OCCUPANCYGRIDFUSION_HPP_
