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
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(palletizer_msgs REQUIRED)

# Planner: pure ROS, no MoveIt C++ API
add_executable(pallet_planner src/pallet_planner.cpp)
target_compile_features(pallet_planner PUBLIC cxx_std_17)
ament_target_dependencies(pallet_planner
  rclcpp std_msgs std_srvs geometry_msgs moveit_msgs shape_msgs visualization_msgs palletizer_msgs)

# Executor: MoveIt
add_executable(palletizer_node src/palletizer_node.cpp)
target_compile_features(palletizer_node PUBLIC cxx_std_17)
ament_target_dependencies(palletizer_node
  rclcpp moveit_ros_planning_interface moveit_msgs geometry_msgs std_msgs palletizer_msgs)

install(TARGETS pallet_planner palletizer_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
