#!/bin/bash

sudo apt update
sudo apt install -y \\
  ros-iron-ament-cmake \\
  ros-iron-grid-map-core \\
  ros-iron-grid-map-ros \\
  ros-iron-grid-map-msgs \\
  ros-iron-grid-map-rviz-plugin \\
  ros-iron-kindr \\
  ros-iron-kindr-ros \\
  ros-iron-message-filters \\
  ros-iron-pcl-ros \\
  ros-iron-rclcpp \\
  ros-iron-sensor-msgs \\
  ros-iron-std-msgs \\
  ros-iron-std-srvs \\
  ros-iron-tf2 \\
  ros-iron-tf2-ros \\
  ros-iron-tf2-eigen \\
  ros-iron-tf2-geometry-msgs \\
  ros-iron-geometry-msgs \\
  libboost-all-dev \\
  libeigen3-dev \\
  libpcl-dev \\
  python3-pip

pip3 install --user colcon-common-extensions
