cmake_minimum_required(VERSION 3.8)
project(palletizer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(palletizer_node src/palletizer_node.cpp)
target_compile_features(palletizer_node PUBLIC cxx_std_17)
ament_target_dependencies(palletizer_node
  rclcpp moveit_ros_planning_interface moveit_msgs geometry_msgs shape_msgs std_msgs)

install(TARGETS palletizer_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
