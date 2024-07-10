#include "grid_map_demos/occupancy_grid_value_node.hpp"

OccupancyGridValueNode::OccupancyGridValueNode()
: Node("occupancy_grid_value_node"),
  occupancy_grid_(nullptr)
{
    point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/clicked_point", 10, std::bind(&OccupancyGridValueNode::pointCallback, this, std::placeholders::_1));

    grid_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/concatenated_map", 10, std::bind(&OccupancyGridValueNode::gridCallback, this, std::placeholders::_1));
}

void OccupancyGridValueNode::gridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received occupancy grid");
    occupancy_grid_ = msg;
}

void OccupancyGridValueNode::pointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    if (!occupancy_grid_) {
        RCLCPP_WARN(this->get_logger(), "Occupancy grid not received yet");
        return;
    }

    double x = msg->point.x;
    double y = msg->point.y;

    int grid_x = static_cast<int>((x - occupancy_grid_->info.origin.position.x) / occupancy_grid_->info.resolution);
    int grid_y = static_cast<int>((y - occupancy_grid_->info.origin.position.y) / occupancy_grid_->info.resolution);

    if (grid_x < 0 || grid_x >= static_cast<int>(occupancy_grid_->info.width) ||
        grid_y < 0 || grid_y >= static_cast<int>(occupancy_grid_->info.height)) {
        RCLCPP_WARN(this->get_logger(), "Clicked point is out of occupancy grid bounds");
        return;
    }

    int index = grid_y * occupancy_grid_->info.width + grid_x;
    int value = occupancy_grid_->data[index];

    RCLCPP_INFO(this->get_logger(), "Clicked point value: %d", value);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OccupancyGridValueNode>());
    rclcpp::shutdown();
    return 0;
}
