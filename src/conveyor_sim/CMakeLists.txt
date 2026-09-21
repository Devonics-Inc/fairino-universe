# min CMAKE Version that is responsible for reading this file 
cmake_minimum_required(VERSION 3.8)

# project name will be needed later as refence
project(conveyor_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# list of packages that i need to have to compile this project
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(conveyor_interfaces REQUIRED)

set(NODES box_spawner box_advancer conveyor_manager proximity_sensor)

foreach(node ${NODES})
  add_executable(${node} src/${node}.cpp)
  target_include_directories(${node} PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  target_compile_features(${node} PUBLIC cxx_std_17)
endforeach()

ament_target_dependencies(box_spawner
  rclcpp geometry_msgs moveit_msgs shape_msgs std_srvs)
ament_target_dependencies(box_advancer
  rclcpp geometry_msgs moveit_msgs conveyor_interfaces)
ament_target_dependencies(conveyor_manager
  rclcpp geometry_msgs std_msgs std_srvs conveyor_interfaces)
ament_target_dependencies(proximity_sensor
  rclcpp geometry_msgs std_msgs moveit_core moveit_ros_planning Eigen3)

install(TARGETS ${NODES}
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
