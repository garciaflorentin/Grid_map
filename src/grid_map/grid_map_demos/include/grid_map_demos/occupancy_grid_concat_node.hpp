#ifndef OCCUPANCY_GRID_CONCAT_NODE_HPP
#define OCCUPANCY_GRID_CONCAT_NODE_HPP

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class OccupancyGridConcatNode : public rclcpp::Node
{
public:
    OccupancyGridConcatNode();

private:
    void gridCallback1(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void gridCallback2(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void concatenateGrids();
    void transformOccupancyGrid(const nav_msgs::msg::OccupancyGrid::SharedPtr input,
                                const geometry_msgs::msg::TransformStamped &transform,
                                nav_msgs::msg::OccupancyGrid &output);
    void concatenateOccupancyGrids(const nav_msgs::msg::OccupancyGrid &map1,
                                   const nav_msgs::msg::OccupancyGrid &map2,
                                   nav_msgs::msg::OccupancyGrid &output);
    void concatenateCells(nav_msgs::msg::OccupancyGrid &grid, int cell_size);
    void applyCellColoring(nav_msgs::msg::OccupancyGrid &grid);


    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub_1_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub_2_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;

    nav_msgs::msg::OccupancyGrid::SharedPtr map1_;
    nav_msgs::msg::OccupancyGrid::SharedPtr map2_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    unsigned int cell_size_; // Taille des cellules pour la segmentation
    double slope_weight_; // Weight for the slope cost map
    double obstacle_weight_; // Weight for the obstacle cost map
};

#endif // OCCUPANCY_GRID_CONCAT_NODE_HPP



// #ifndef OCCUPANCY_GRID_CONCAT_NODE_HPP
// #define OCCUPANCY_GRID_CONCAT_NODE_HPP

// #include <memory>
// #include "rclcpp/rclcpp.hpp"
// #include "nav_msgs/msg/occupancy_grid.hpp"
// #include "tf2_ros/transform_listener.h"
// #include "tf2_ros/buffer.h"
// #include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// class OccupancyGridConcatNode : public rclcpp::Node
// {
// public:
//     OccupancyGridConcatNode();

// private:
//     void gridCallback1(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
//     void gridCallback2(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
//     void concatenateGrids();
//     void transformOccupancyGrid(const nav_msgs::msg::OccupancyGrid::SharedPtr input,
//                                 const geometry_msgs::msg::TransformStamped &transform,
//                                 nav_msgs::msg::OccupancyGrid &output);
//     void concatenateOccupancyGrids(const nav_msgs::msg::OccupancyGrid &map1,
//                                    const nav_msgs::msg::OccupancyGrid &map2,
//                                    nav_msgs::msg::OccupancyGrid &output);
//     void applyCellColoring(nav_msgs::msg::OccupancyGrid &grid);

//     rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub_1_;
//     rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub_2_;
//     rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;

//     nav_msgs::msg::OccupancyGrid::SharedPtr map1_;
//     nav_msgs::msg::OccupancyGrid::SharedPtr map2_;

//     std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
//     std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

//     unsigned int cell_size_; // Taille des cellules carrées
// };

// #endif // OCCUPANCY_GRID_CONCAT_NODE_HPP

