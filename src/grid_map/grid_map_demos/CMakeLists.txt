cmake_minimum_required(VERSION 3.5.0)
project(grid_map_demos)

## Find ament_cmake macros and libraries
find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(filters REQUIRED)
find_package(grid_map_cmake_helpers REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_cv REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_octomap REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(OCTOMAP REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(CURL REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(OpenCV REQUIRED
  COMPONENTS
  opencv_highgui
  CONFIG
)

grid_map_package()

set(dependencies
  grid_map_msgs
  grid_map_ros
  rclcpp
  cv_bridge
  geometry_msgs
  grid_map_core
  grid_map_cv
  nav2_msgs
  nav_msgs
  rcutils
  rosbag2_cpp
  sensor_msgs
  std_msgs
  tf2
  visualization_msgs
  tf2_msgs
  tf2_geometry_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  SYSTEM
    ${EIGEN3_INCLUDE_DIR}
)
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib:${CMAKE_INSTALL_PREFIX}/lib64:/usr/lib/x86_64-linux-gnu")

## Declare a cpp executable


##TF PUBLISHER FOR SIMU####

add_executable(
  transform_publisher_node 
  src/transform_publisher_node.cpp
)

add_executable(
  tf_static_publisher 
  src/tf_static_publisher.cpp
)

###########################

add_executable(
  filters_demo
  src/filters_demo_node.cpp
)

add_library(filters_demo_lib SHARED
  src/FiltersDemo.cpp
)


target_link_libraries(filters_demo CURL::libcurl)



## Specify libraries to link a library or executable target against


ament_target_dependencies(
  transform_publisher_node SYSTEM
  ${dependencies}
)

ament_target_dependencies(
  tf_static_publisher SYSTEM
  ${dependencies}
)

target_link_libraries(filters_demo filters_demo_lib)

ament_target_dependencies(
  filters_demo_lib SYSTEM
  ${dependencies}
  filters
)

ament_target_dependencies(
  filters_demo SYSTEM
  ${dependencies}
  filters
)


set(targets_list
  filters_demo
  filters_demo_lib
  transform_publisher_node
  tf_static_publisher

)


foreach(target ${targets_list})
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    target_compile_options(${target} PRIVATE "SHELL:--param ggc-min-expand=1")
    target_compile_options(${target} PRIVATE "SHELL:--param ggc-min-heapsize=32768")
  endif()
endforeach()




#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
  TARGETS ${targets_list}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Mark other files for installation
install(
  DIRECTORY config data doc launch rviz
  DESTINATION share/${PROJECT_NAME}
)

# Install python scripts
install(
  PROGRAMS scripts/image_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  # Linting is setup this way to add a filter
  # to ament_cpplint to ignore the lack of
  # copyright messages at the top of files.
  # Copyright messages are being checked for by both
  # ament_cmake_cpplint & ament_cmake_copyright.

  find_package(ament_lint_auto REQUIRED)
  find_package(ament_lint_auto QUIET)
  if(ament_lint_auto_FOUND)
    # exclude copyright checks
    list(APPEND AMENT_LINT_AUTO_EXCLUDE
      ament_cmake_cpplint
      ament_cmake_copyright
    )
    ament_lint_auto_find_test_dependencies()

    # run cpplint without copyright filter
    find_package(ament_cmake_cpplint)
    ament_cpplint(
      FILTERS -legal/copyright -build/include_order
    )
  endif()
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include ${EIGEN3_INCLUDE_DIR})
ament_package()
