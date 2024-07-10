#ifndef FLOOR_NORMAL_HPP_
#define FLOOR_NORMAL_HPP_

#include <rclcpp/rclcpp.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <filters/filter_chain.hpp>
#include <string>

class floorNormal : public rclcpp::Node
{
public:
    floorNormal();
    virtual ~floorNormal();
    bool readParameters();
    void floorNormalCallback(const grid_map_msgs::msg::GridMap::SharedPtr msg);


private:
    std::string inputTopic_;

    std::string outputTopic_;

    rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr subscription_;
    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr publisher_;


    filters::FilterChain<grid_map::GridMap> filterChain_;

    std::string filterChainParametersName_;
    
};

#endif // FLOOR_NORMAL_HPP_
