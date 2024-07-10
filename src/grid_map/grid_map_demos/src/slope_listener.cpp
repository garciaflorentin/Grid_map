#include <rclcpp/rclcpp.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <iostream>

class SlopeListener : public rclcpp::Node
{
public:
    SlopeListener()
    : Node("slope_listener")
    {
        subscription_ = this->create_subscription<grid_map_msgs::msg::GridMap>(
            "/filtered_map", 10, std::bind(&SlopeListener::slopeListenerCallback, this, std::placeholders::_1));
        
        publisher_ = this->create_publisher<grid_map_msgs::msg::GridMap>("slope", 10);
    }

private:
    void slopeListenerCallback(const grid_map_msgs::msg::GridMap::SharedPtr msg)
    {
        grid_map::GridMap map;
        grid_map::GridMapRosConverter::fromMessage(*msg, map);

        std::string layer_name = "slope";  

        if (map.exists(layer_name)) {
            const grid_map::Matrix& data = map[layer_name];

            for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
                const grid_map::Index index(*it);
                // float value = data(index(0), index(1));
                // Suppression des impressions
                // RCLCPP_INFO(this->get_logger(), "Value at (%d, %d) = %f", index(0), index(1), value);
            }

            std::unique_ptr<grid_map_msgs::msg::GridMap> outputMessage;
            outputMessage = grid_map::GridMapRosConverter::toMessage(map);
            publisher_->publish(std::move(outputMessage));

        } else {
            // Suppression des impressions
            // RCLCPP_ERROR(this->get_logger(), "Layer %s not found in the grid map.", layer_name.c_str());
        }
    }

    rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr subscription_;
    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SlopeListener>());
    rclcpp::shutdown();
    return 0;
}
