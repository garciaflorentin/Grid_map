
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include "grid_map_demos/FiltersDemo.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<grid_map_demos::FiltersDemo>());
  rclcpp::shutdown();
  return 0;
}
