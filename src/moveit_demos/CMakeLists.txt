cmake_minimum_required(VERSION 3.8)
project(moveit_demos)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)



add_executable(MtDm1 src/MtDm1.cpp)
ament_target_dependencies(MtDm1 rclcpp moveit_ros_planning_interface geometry_msgs)
install(TARGETS MtDm1 DESTINATION lib/${PROJECT_NAME})


# add_executable(robot_model_and_robot_state src/robot_model_and_robot_state.cpp)
# ament_target_dependencies(robot_model_and_robot_state rclcpp moveit_ros_planning_interface)
# install(TARGETS robot_model_and_robot_state DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
