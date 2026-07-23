cmake_minimum_required(VERSION 3.8)
project(fairino_gripper_integrated_desc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install the directories visible in image_8fa87b.png
install(DIRECTORY
  meshes
  urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
