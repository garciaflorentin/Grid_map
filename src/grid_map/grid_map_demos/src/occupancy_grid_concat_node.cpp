#include "grid_map_demos/occupancy_grid_concat_node.hpp"

OccupancyGridConcatNode::OccupancyGridConcatNode()
: Node("occupancy_grid_concat_node"),
  map1_(nullptr),
  map2_(nullptr),
  cell_size_(3), // Taille de la cellule en nombre de pixels
  slope_weight_(0.5), // Example weight for slope cost map
  obstacle_weight_(0.5) // Example weight for obstacle cost map
{
    grid_sub_1_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/traversability_grid", 10, std::bind(&OccupancyGridConcatNode::gridCallback1, this, std::placeholders::_1));
    
    grid_sub_2_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/occupancy_grid_obstacle", 10, std::bind(&OccupancyGridConcatNode::gridCallback2, this, std::placeholders::_1));
    
    grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/concatenated_map", 10);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void OccupancyGridConcatNode::gridCallback1(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    map1_ = msg;
    if (map2_) concatenateGrids();
}

void OccupancyGridConcatNode::gridCallback2(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    map2_ = msg;
    if (map1_) concatenateGrids();
}

void OccupancyGridConcatNode::concatenateGrids()
{
    geometry_msgs::msg::TransformStamped transformStamped;
    try
    {
        transformStamped = tf_buffer_->lookupTransform(map1_->header.frame_id, map2_->header.frame_id, tf2::TimePointZero);
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
        return;
    }

    nav_msgs::msg::OccupancyGrid transformed_map2 = *map2_;
    transformOccupancyGrid(map2_, transformStamped, transformed_map2);

    nav_msgs::msg::OccupancyGrid concatenated_map;
    concatenateOccupancyGrids(*map1_, transformed_map2, concatenated_map);

    // concatenateCells(concatenated_map, cell_size_);
    applyCellColoring(concatenated_map);


    concatenated_map.header.frame_id = map1_->header.frame_id;

    grid_pub_->publish(concatenated_map);
}

void OccupancyGridConcatNode::transformOccupancyGrid(const nav_msgs::msg::OccupancyGrid::SharedPtr input,
                                                     const geometry_msgs::msg::TransformStamped &transform,
                                                     nav_msgs::msg::OccupancyGrid &output)
{
    output = *input;

    tf2::Transform tf_transform;
    tf2::fromMsg(transform.transform, tf_transform);

    for (unsigned int y = 0; y < input->info.height; ++y)
    {
        for (unsigned int x = 0; x < input->info.width; ++x)
        {
            int index = y * input->info.width + x;

            if (input->data[index] == -1)
                continue;

            double wx = input->info.origin.position.x + x * input->info.resolution;
            double wy = input->info.origin.position.y + y * input->info.resolution;

            tf2::Vector3 point(wx, wy, 0);
            tf2::Vector3 transformed_point = tf_transform * point;

            unsigned int new_x = static_cast<unsigned int>((transformed_point.x() - output.info.origin.position.x) / output.info.resolution);
            unsigned int new_y = static_cast<unsigned int>((transformed_point.y() - output.info.origin.position.y) / output.info.resolution);

            if (new_x < output.info.width && new_y < output.info.height)
            {
                unsigned int new_index = new_y * output.info.width + new_x;
                output.data[new_index] = input->data[index];
            }
        }
    }
}
void OccupancyGridConcatNode::concatenateOccupancyGrids(const nav_msgs::msg::OccupancyGrid &map1,
                                                        const nav_msgs::msg::OccupancyGrid &map2,
                                                        nav_msgs::msg::OccupancyGrid &output)
{
    double resolution = std::min(map1.info.resolution, map2.info.resolution);

    double min_x = std::min(map1.info.origin.position.x, map2.info.origin.position.x);
    double min_y = std::min(map1.info.origin.position.y, map2.info.origin.position.y);
    double max_x = std::max(map1.info.origin.position.x + map1.info.width * map1.info.resolution,
                            map2.info.origin.position.x + map2.info.width * map2.info.resolution);
    double max_y = std::max(map1.info.origin.position.y + map1.info.height * map1.info.resolution,
                            map2.info.origin.position.y + map2.info.height * map2.info.resolution);

    output.info.resolution = resolution;
    output.info.width = static_cast<uint32_t>((max_x - min_x) / resolution);
    output.info.height = static_cast<uint32_t>((max_y - min_y) / resolution);
    output.info.origin.position.x = min_x;
    output.info.origin.position.y = min_y;
    output.info.origin.position.z = 0.0;
    output.info.origin.orientation.w = 1.0;
    output.info.origin.orientation.x = 0.0;
    output.info.origin.orientation.y = 0.0;
    output.info.origin.orientation.z = 0.0;

    output.data.resize(output.info.width * output.info.height, -1);

    for (unsigned int y = 0; y < map1.info.height; ++y)
    {
        for (unsigned int x = 0; x < map1.info.width; ++x)
        {
            int index = y * map1.info.width + x;
            if (map1.data[index] == -1)
                continue;

            double wx = map1.info.origin.position.x + x * map1.info.resolution;
            double wy = map1.info.origin.position.y + y * map1.info.resolution;

            unsigned int new_x = static_cast<unsigned int>((wx - min_x) / resolution);
            unsigned int new_y = static_cast<unsigned int>((wy - min_y) / resolution);

            unsigned int new_index = new_y * output.info.width + new_x;
            // output.data[new_index] = map1.data[index];
            output.data[new_index] = static_cast<int>(map1.data[index] * slope_weight_);
        }
    }

    for (unsigned int y = 0; y < map2.info.height; ++y)
    {
        for (unsigned int x = 0; x < map2.info.width; ++x)
        {
            int index = y * map2.info.width + x;
            if (map2.data[index] == -1)
                continue;

            double wx = map2.info.origin.position.x + x * map2.info.resolution;
            double wy = map2.info.origin.position.y + y * map2.info.resolution;

            unsigned int new_x = static_cast<unsigned int>((wx - min_x) / resolution);
            unsigned int new_y = static_cast<unsigned int>((wy - min_y) / resolution);

            unsigned int new_index = new_y * output.info.width + new_x;

            if (output.data[new_index] == -1)
            {
                // output.data[new_index] = map2.data[index];
                output.data[new_index] = static_cast<int>(map2.data[index] * obstacle_weight_);
            }
            else
            {
                int val= output.data[new_index] + static_cast<int>(map2.data[index] * obstacle_weight_);
                if ( val > 100)
                {
                    output.data[new_index] = 100;
                }
                else
                {
                    output.data[new_index] = val;
                }
            }
        }
    }

    for (unsigned int i = 0; i < output.data.size(); ++i)
    {
        if (output.data[i] == 0)
        {
            output.data[i] = 1;
        }
    }
}

void OccupancyGridConcatNode::concatenateCells(nav_msgs::msg::OccupancyGrid &grid, int cell_size)
{
    int width = grid.info.width;
    int height = grid.info.height;
    std::vector<int8_t> new_data = grid.data;

    for (int y = 0; y < height; y += cell_size)
    {
        for (int x = 0; x < width; x += cell_size)
        {
            std::unordered_map<int, int> value_count;

            for (int j = 0; j < cell_size && (y + j) < height; ++j)
            {
                for (int i = 0; i < cell_size && (x + i) < width; ++i)
                {
                    int index = (y + j) * width + (x + i);
                    int value = grid.data[index];
                    if (value >= 0)
                    {
                        value_count[value]++;
                    }
                }
            }

            int majority_value = -1;
            int max_count = 0;
            for (const auto &pair : value_count)
            {
                if (pair.second > max_count)
                {
                    majority_value = pair.first;
                    max_count = pair.second;
                }
            }

            for (int j = 0; j < cell_size && (y + j) < height; ++j)
            {
                for (int i = 0; i < cell_size && (x + i) < width; ++i)
                {
                    int index = (y + j) * width + (x + i);
                    new_data[index] = majority_value;
                }
            }
        }
    }

    grid.data = new_data;
}

void OccupancyGridConcatNode::applyCellColoring(nav_msgs::msg::OccupancyGrid &grid)
{
    // Parcourir la grille par blocs de cellules de taille cell_size_
    for (unsigned int y = 0; y < grid.info.height; y += cell_size_)
    {
        for (unsigned int x = 0; x < grid.info.width; x += cell_size_)
        {
            // Trouver la valeur maximale dans le bloc de cellules
            int max_value = -1;
            for (unsigned int dy = 0; dy < cell_size_; ++dy)
            {
                for (unsigned int dx = 0; dx < cell_size_; ++dx)
                {
                    unsigned int index = (y + dy) * grid.info.width + (x + dx);
                    if (index < grid.data.size() && grid.data[index] > max_value)
                    {
                        max_value = grid.data[index];
                    }
                }
            }

            // Appliquer la valeur maximale à toutes les cellules du bloc
            for (unsigned int dy = 0; dy < cell_size_; ++dy)
            {
                for (unsigned int dx = 0; dx < cell_size_; ++dx)
                {
                    unsigned int index = (y + dy) * grid.info.width + (x + dx);
                    if (index < grid.data.size())
                    {
                        grid.data[index] = max_value;
                    }
                }
            }
        }
    }
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OccupancyGridConcatNode>());
    rclcpp::shutdown();
    return 0;
}


// #include "grid_map_demos/occupancy_grid_concat_node.hpp"

// OccupancyGridConcatNode::OccupancyGridConcatNode()
// : Node("occupancy_grid_concat_node"),
//   map1_(nullptr),
//   map2_(nullptr),
//   cell_size_(5) // Taille des cellules carrées
// {
//     grid_sub_1_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
//         "/traversability_grid", 10, std::bind(&OccupancyGridConcatNode::gridCallback1, this, std::placeholders::_1));
    
//     grid_sub_2_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
//         "/occupancy_grid_obstacle", 10, std::bind(&OccupancyGridConcatNode::gridCallback2, this, std::placeholders::_1));
    
//     grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/concatenated_map", 10);

//     tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
//     tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
// }

// void OccupancyGridConcatNode::gridCallback1(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
// {
//     map1_ = msg;
//     if (map2_) concatenateGrids();
// }

// void OccupancyGridConcatNode::gridCallback2(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
// {
//     map2_ = msg;
//     if (map1_) concatenateGrids();
// }

// void OccupancyGridConcatNode::concatenateGrids()
// {
//     geometry_msgs::msg::TransformStamped transformStamped;
//     try
//     {
//         transformStamped = tf_buffer_->lookupTransform(map1_->header.frame_id, map2_->header.frame_id, tf2::TimePointZero);
//     }
//     catch (tf2::TransformException &ex)
//     {
//         RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
//         return;
//     }

//     nav_msgs::msg::OccupancyGrid transformed_map2 = *map2_;
//     transformOccupancyGrid(map2_, transformStamped, transformed_map2);

//     nav_msgs::msg::OccupancyGrid concatenated_map;
//     concatenateOccupancyGrids(*map1_, transformed_map2, concatenated_map);

//     applyCellColoring(concatenated_map);

//     concatenated_map.header.frame_id = map1_->header.frame_id;

//     grid_pub_->publish(concatenated_map);
// }

// void OccupancyGridConcatNode::transformOccupancyGrid(const nav_msgs::msg::OccupancyGrid::SharedPtr input,
//                                                      const geometry_msgs::msg::TransformStamped &transform,
//                                                      nav_msgs::msg::OccupancyGrid &output)
// {
//     output = *input;

//     tf2::Transform tf_transform;
//     tf2::fromMsg(transform.transform, tf_transform);

//     for (unsigned int y = 0; y < input->info.height; ++y)
//     {
//         for (unsigned int x = 0; x < input->info.width; ++x)
//         {
//             int index = y * input->info.width + x;

//             if (input->data[index] == -1)
//                 continue;

//             double wx = input->info.origin.position.x + x * input->info.resolution;
//             double wy = input->info.origin.position.y + y * input->info.resolution;

//             tf2::Vector3 point(wx, wy, 0);
//             tf2::Vector3 transformed_point = tf_transform * point;

//             unsigned int new_x = static_cast<unsigned int>((transformed_point.x() - output.info.origin.position.x) / output.info.resolution);
//             unsigned int new_y = static_cast<unsigned int>((transformed_point.y() - output.info.origin.position.y) / output.info.resolution);

//             if (new_x < output.info.width && new_y < output.info.height)
//             {
//                 unsigned int new_index = new_y * output.info.width + new_x;
//                 output.data[new_index] = input->data[index];
//             }
//         }
//     }
// }

// void OccupancyGridConcatNode::concatenateOccupancyGrids(const nav_msgs::msg::OccupancyGrid &map1,
//                                                         const nav_msgs::msg::OccupancyGrid &map2,
//                                                         nav_msgs::msg::OccupancyGrid &output)
// {
//     double resolution = std::min(map1.info.resolution, map2.info.resolution);

//     double min_x = std::min(map1.info.origin.position.x, map2.info.origin.position.x);
//     double min_y = std::min(map1.info.origin.position.y, map2.info.origin.position.y);
//     double max_x = std::max(map1.info.origin.position.x + map1.info.width * map1.info.resolution,
//                             map2.info.origin.position.x + map2.info.width * map2.info.resolution);
//     double max_y = std::max(map1.info.origin.position.y + map1.info.height * map1.info.resolution,
//                             map2.info.origin.position.y + map2.info.height * map2.info.resolution);

//     output.info.resolution = resolution;
//     output.info.width = static_cast<uint32_t>((max_x - min_x) / resolution);
//     output.info.height = static_cast<uint32_t>((max_y - min_y) / resolution);
//     output.info.origin.position.x = min_x;
//     output.info.origin.position.y = min_y;
//     output.info.origin.position.z = 0.0;
//     output.info.origin.orientation.w = 1.0;
//     output.info.origin.orientation.x = 0.0;
//     output.info.origin.orientation.y = 0.0;
//     output.info.origin.orientation.z = 0.0;

//     output.data.resize(output.info.width * output.info.height, -1);

//     for (unsigned int y = 0; y < map1.info.height; ++y)
//     {
//         for (unsigned int x = 0; x < map1.info.width; ++x)
//         {
//             int index = y * map1.info.width + x;
//             if (map1.data[index] == -1)
//                 continue;

//             double wx = map1.info.origin.position.x + x * map1.info.resolution;
//             double wy = map1.info.origin.position.y + y * map1.info.resolution;

//             unsigned int new_x = static_cast<unsigned int>((wx - min_x) / resolution);
//             unsigned int new_y = static_cast<unsigned int>((wy - min_y) / resolution);

//             unsigned int new_index = new_y * output.info.width + new_x;
//             output.data[new_index] = map1.data[index];
//         }
//     }

//     for (unsigned int y = 0; y < map2.info.height; ++y)
//     {
//         for (unsigned int x = 0; x < map2.info.width; ++x)
//         {
//             int index = y * map2.info.width + x;
//             if (map2.data[index] == -1)
//                 continue;

//             double wx = map2.info.origin.position.x + x * map2.info.resolution;
//             double wy = map2.info.origin.position.y + y * map2.info.resolution;

//             unsigned int new_x = static_cast<unsigned int>((wx - min_x) / resolution);
//             unsigned int new_y = static_cast<unsigned int>((wy - min_y) / resolution);

//             unsigned int new_index = new_y * output.info.width + new_x;

//             if (output.data[new_index] == -1)
//             {
//                 output.data[new_index] = map2.data[index];
//             }
//             else
//             {
//                 output.data[new_index] += map2.data[index];
//                 if (output.data[new_index] > 100)
//                 {
//                     output.data[new_index] = 100;
//                 }
//             }
//         }
//     }

//     for (unsigned int i = 0; i < output.data.size(); ++i)
//     {
//         if (output.data[i] == 0)
//         {
//             output.data[i] = 1;
//         }
//     }
// }

// void OccupancyGridConcatNode::applyCellColoring(nav_msgs::msg::OccupancyGrid &grid)
// {
//     // Parcourir la grille par blocs de cellules de taille cell_size_
//     for (unsigned int y = 0; y < grid.info.height; y += cell_size_)
//     {
//         for (unsigned int x = 0; y < grid.info.width; x += cell_size_)
//         {
//             // Trouver la valeur maximale dans le bloc de cellules
//             int max_value = -1;
//             for (unsigned int dy = 0; dy < cell_size_; ++dy)
//             {
//                 for (unsigned int dx = 0; dx < cell_size_; ++dx)
//                 {
//                     unsigned int index = (y + dy) * grid.info.width + (x + dx);
//                     if (index < grid.data.size() && grid.data[index] > max_value)
//                     {
//                         max_value = grid.data[index];
//                     }
//                 }
//             }

//             // Appliquer la valeur maximale à toutes les cellules du bloc
//             for (unsigned int dy = 0; dy < cell_size_; ++dy)
//             {
//                 for (unsigned int dx = 0; dx < cell_size_; ++dx)
//                 {
//                     unsigned int index = (y + dy) * grid.info.width + (x + dx);
//                     if (index < grid.data.size())
//                     {
//                         grid.data[index] = max_value;
//                     }
//                 }
//             }
//         }
//     }
// }

// int main(int argc, char **argv)
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<OccupancyGridConcatNode>());
//     rclcpp::shutdown();
//     return 0;
// }
