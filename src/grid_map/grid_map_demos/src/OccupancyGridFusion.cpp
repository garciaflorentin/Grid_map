#include "grid_map_demos/OccupancyGridFusion.hpp"

OccupancyGridFusion::OccupancyGridFusion()
    : Node("occupancy_grid_fusion"),weight1_(2), weight2_(0.5)
{
    grid1_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/traversability_grid", 10,
        std::bind(&OccupancyGridFusion::grid1Callback, this, std::placeholders::_1));

    grid2_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/occupancy_grid_obstacle", 10,
        std::bind(&OccupancyGridFusion::grid2Callback, this, std::placeholders::_1));

    fused_grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/fused_grid", 10);
}

void OccupancyGridFusion::grid1Callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    grid1_ = msg;
    if (grid2_) {
        fuseGrids();
    }
}

void OccupancyGridFusion::grid2Callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    grid2_ = msg;
    if (grid1_) {
        fuseGrids();
    }
}

int OccupancyGridFusion::mapValue(int value, int min_in, int max_in, int min_out, int max_out)
{
    return min_out + (value - min_in) * (max_out - min_out) / (max_in - min_in);
}

void OccupancyGridFusion::fuseGrids()
{
    // Prendre les dimensions et la résolution minimales
    int min_width = std::min(grid1_->info.width, grid2_->info.width);
    int min_height = std::min(grid1_->info.height, grid2_->info.height);
    double min_resolution = std::min(grid1_->info.resolution, grid2_->info.resolution);

    nav_msgs::msg::OccupancyGrid fused_grid;
    fused_grid.header.stamp = this->now();
    fused_grid.header.frame_id = grid1_->header.frame_id;
    fused_grid.info.resolution = min_resolution;
    fused_grid.info.width = min_width;
    fused_grid.info.height = min_height;
    fused_grid.info.origin = grid1_->info.origin;

    fused_grid.data.resize(min_width * min_height, -1);

    for (int y = 0; y < min_height; ++y) {
        for (int x = 0; x < min_width; ++x) {
            int index = y * min_width + x;
            int index1 = y * grid1_->info.width + x;
            int index2 = y * grid2_->info.width + x;

            int value1 = grid1_->data[index1];
            int value2 = grid2_->data[index2];

            if (value1 != -1) {
                std::cout<<"value avant mapping = "<<value1<<std::endl;
                value1 = mapValue(value1, 0, 100, 0, 100); // Map value1 to range 0-100
                std::cout<<"value apres mapping = "<<value1<<std::endl;

            }
            
            if (value1 == -1 && value2 == -1) {
                fused_grid.data[index] = -1; // Unknown
            } else if (value1 == -1) {
                fused_grid.data[index] = value2;
            } else if (value2 == -1) {
                fused_grid.data[index] = value1;
            } else {
                fused_grid.data[index] = static_cast<int>(value1 * weight1_ + value2 * weight2_);
            }
        }
    }

    fused_grid_pub_->publish(fused_grid);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OccupancyGridFusion>());
    rclcpp::shutdown();
    return 0;
}
