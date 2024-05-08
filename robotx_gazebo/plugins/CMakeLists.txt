cmake_minimum_required(VERSION 3.10)
project(marker_plugins)

# Find packages
find_package(gazebo REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${rclcpp_INCLUDE_DIRS})

# Add plugins
add_library(move_in_circle_plugin SHARED MoveInCirclePlugin.cpp)
target_link_libraries(move_in_circle_plugin ${GAZEBO_LIBRARIES})

