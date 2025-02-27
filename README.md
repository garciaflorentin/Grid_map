# Grid Map for ROS2 Iron

## Overview

This repository is a re-appropriation of the original Grid Map repository, translated to ROS2 Iron. The grid_map package is designed to apply mathematical operations and filters to the elevation map created by the elevation_mapping package. This provides enhanced information about the geometry of the ground, which is crucial for autonomous navigation in complex environments.

The primary purpose of this package is to evaluate the traversability of terrain for an autonomous ground robot using Lidar and RGB data. The key launch file for running the package is filters_demo_launch.py, which processes a grid map input and outputs a filtered grid map with various layers that provide different information about the ground geometry. Additionally, RViz2 is launched to visualize the results.

## Features

- Multi-layered grid maps: Supports multiple data layers such as elevation, variance, color, friction coefficient, foothold quality, surface normal, and traversability.
- Efficient map repositioning: Uses a two-dimensional circular buffer for non-destructive map shifting.
- Eigen-based data storage: Grid map data is stored as Eigen data types, allowing direct application of Eigen algorithms for versatile data manipulation.
- Convenience functions: Provides helper methods for safe and convenient data access, including iterator functions for various regions and shapes.
- ROS interface: Direct conversion between grid maps and ROS message types such as PointCloud2, OccupancyGrid, GridCells, and custom GridMap messages.
- OpenCV interface: Seamless conversion between grid maps and OpenCV image types.
- Visualizations: RViz plugins for 3D surface plots and point cloud visualizations.
- Filters: A range of filters for processing grid maps, including thresholding, normal vector computation, smoothening, variance, inpainting, and matrix kernel convolutions.

## Installation

### Clone the Repository:

```
git clone https://github.com/garciaflorentin/Grid_map.git
cd grid_map
```

### Install Dependencies:
Use the provided script to install dependencies:

```
./install_dependencies.sh
```

### Build the Package:
Ensure you are in the root of your workspace and then build the package using colcon:

```
colcon build --symlink-install
```

### Source the Setup File:
After building, source the setup file to overlay this workspace on your environment:

```
source install/setup.bash
```

## Usage

### Running the Filter Demo

To run the filter demo, which processes a grid map to extract various geometric features of the ground, use the following command:

```
ros2 launch grid_map_demos filters_demo_launch.py
```

This launch file will start the filtering process and open RViz2 to visualize the results.

### Inputs and Outputs

- Input: The filter demo node takes a grid map as input, typically representing an elevation map generated by the elevation_mapping package.
- Output: The node outputs a filtered grid map with multiple layers, each providing different information about the ground geometry such as surface normals, slopes, and other features.

### Example Workflow

Here is a step-by-step guide to using the grid map package:

1. **Launch the Filter Demo**:

```
    ros2 launch grid_map_demos filters_demo_launch.py
```

2. **Provide Grid Map Data**:
   Ensure that the appropriate grid map data is being published to the expected topics.

3. **Monitor Outputs in RViz**:
   The results of the filtering process will be visualized in RViz, providing detailed information about the ground geometry.

## Package Structure

The repository is structured as follows:

- **grid_map**: Contains the main grid map library and related nodes.
  - \`include/grid_map\`: Header files for the grid map library.
  - \`src\`: Source files for the grid map library and nodes.
  - \`CMakeLists.txt\`: Build configuration for the grid map package.
  - \`package.xml\`: Package configuration file for ROS2.

- **grid_map_demos**: Contains demo launch files, configurations, and scripts.
  - \`config\`: Configuration files for the demos.
  - \`launch\`: Launch files to start the grid map demos.
  - \`rviz\`: RViz configuration files.
  - \`scripts\`: Helper scripts.
  - \`CMakeLists.txt\`: Build configuration for the demo package.
  - \`package.xml\`: Package configuration file for ROS2.

## Configuration

The configuration files are located in the \`config\` directory of \`grid_map_demos\`. You can customize these files to fit your specific use case.

### Example Configuration Files

#### Filter Chain Configuration

```
filters_demo_filter_chain.yaml:
  filter_chain:
    - name: "lower_threshold"
      type: "gridMapFilters/ThresholdFilter"
      params:
        layer: "elevation"
        lower_threshold: 0.0
        set_to: 0.0
    - name: "mean_in_radius"
      type: "gridMapFilters/MeanInRadiusFilter"
      params:
        input_layer: "elevation"
        output_layer: "elevation_smooth"
        radius: 0.06
    - name: "normal_vectors"
      type: "gridMapFilters/NormalVectorsFilter"
      params:
        input_layer: "elevation_smooth"
        output_layers_prefix: "normal_"
        radius: 0.05
        normal_vector_positive_axis: "z"
    - name: "normal_color"
      type: "gridMapFilters/NormalColorMapFilter"
      params:
        input_layers_prefix: "normal_"
        output_layer: "normal_color"
```
