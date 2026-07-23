cmake_minimum_required(VERSION 3.8)
project(fairino_hardware_v3_9_5a)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fPIC -g -ggdb3)
endif()

#set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(fairino_msgs REQUIRED)


#set variable
set(SRC_DIR EXP_SRC_DIR)
file(GLOB SRC_DIR ./src/command_server.cpp ./src/command_server_node.cpp ./src/CNDE_thread.cpp ./src/global_val_def.cpp)
file(GLOB EXP_SRC_DIR ./examples/src/*.cpp)

#compile
# file(CREATE_LINK 
# "${CMAKE_CURRENT_SOURCE_DIR}/libfairino/lib/libfairino.so.2.3.0"
# "${CMAKE_CURRENT_SOURCE_DIR}/libfairino/lib/libfairino.so.2"
# SYMBOLIC)

# file(CREATE_LINK 
# "${CMAKE_CURRENT_SOURCE_DIR}/libfairino/lib/libfairino.so.2"
# "${CMAKE_CURRENT_SOURCE_DIR}/libfairino/lib/libfairino.so"
# SYMBOLIC)

add_executable(ros2_cmd_server ${SRC_DIR})
add_executable(srv_test_client ${EXP_SRC_DIR})
add_library(
  fairino_hardware 
  SHARED 
  ./src/fairino_hardware_interface.cpp
)

#target_compile_definitions(ros2_cmd_server PRIVATE CHN_VERSION)
#target_compile_definitions(ros2_cmd_server PRIVATE ENG_VERSION)


#add headfile source
target_include_directories(ros2_cmd_server PUBLIC 
${CMAKE_CURRENT_SOURCE_DIR}/include/ 
${CMAKE_CURRENT_SOURCE_DIR}/)

target_include_directories(srv_test_client PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/examples/include)
target_include_directories(fairino_hardware PUBLIC 
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/>
$<INSTALL_INTERFACE:include/fairino_hardware>
)
# target_link_libraries(ros2_cmd_server ${CMAKE_CURRENT_SOURCE_DIR}/libfairino/lib/libfairino.so)
# target_link_libraries(fairino_hardware ${CMAKE_CURRENT_SOURCE_DIR}/libfairino/lib/libfairino.so)
# target_link_libraries(ros2_cmd_server ${CMAKE_CURRENT_SOURCE_DIR}/libfairino/lib/libfairino.so.2.3.0)
# target_link_libraries(fairino_hardware ${CMAKE_CURRENT_SOURCE_DIR}/libfairino/lib/libfairino.so.2.3.0)

# 链接库时添加rpath参数
target_link_libraries(ros2_cmd_server "-Wl,-rpath,$ORIGIN" ${CMAKE_CURRENT_SOURCE_DIR}/libfairino/lib/libfairino.so.2.3.5)
target_link_libraries(fairino_hardware "-Wl,-rpath,$ORIGIN" ${CMAKE_CURRENT_SOURCE_DIR}/libfairino/lib/libfairino.so.2.3.5)

#add dependencies
ament_target_dependencies(ros2_cmd_server rclcpp fairino_msgs)
ament_target_dependencies(srv_test_client rclcpp fairino_msgs)
ament_target_dependencies(fairino_hardware 
  hardware_interface
  pluginlib
  rclcpp
  fairino_msgs
  rclcpp_lifecycle 
)


#export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface fairino_hardware.xml)

#install all files
#command server
install(TARGETS ros2_cmd_server
  DESTINATION lib/${PROJECT_NAME}
)

#test example
install(TARGETS srv_test_client
  DESTINATION lib/${PROJECT_NAME}
)

#ros2_control hardware
install(TARGETS fairino_hardware
  EXPORT fairino_hardware
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)



#file(COPY_FILE "libfairino/lib/libfairino.so.2.3.5" 
#"${CMAKE_INSTALL_PREFIX}/lib/libfairino.so.2.3.5" 
#)

# file(CREATE_LINK 
# "${CMAKE_INSTALL_PREFIX}/lib/libfairino.so.2.3.0" 
# "${CMAKE_INSTALL_PREFIX}/lib/libfairino.so.2" 
# SYMBOLIC)

# file(CREATE_LINK 
# "${CMAKE_INSTALL_PREFIX}/lib/libfairino.so.2" 
# "${CMAKE_INSTALL_PREFIX}/lib/libfairino.so" 
# SYMBOLIC)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(hardware_interface REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# --- SMART, ZERO-HARDCODING BASH ENVIRONMENT HOOK & SYMLINK CREATION ---

# 1. Define raw, relocatable shell scripting logic using your exact project variables
set(HOOK_CONTENT "
# Dynamic lookup generated automatically by CMake for ${PROJECT_NAME}
if [ -n \"\$COLCON_CURRENT_PREFIX\" ]; then
    export LD_LIBRARY_PATH=\"\$LD_LIBRARY_PATH:\$COLCON_CURRENT_PREFIX/lib/${PROJECT_NAME}\"
fi
")

# 2. Command CMake to print this snippet into a dynamic script file inside the build directory
set(GENERATED_HOOK_FILE "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_lib_hook.sh")
file(WRITE "${GENERATED_HOOK_FILE}" "${HOOK_CONTENT}")

# 3. Register the freshly generated .sh script as a native environment hook
ament_environment_hooks("${GENERATED_HOOK_FILE}")

# 4. Securely install the primary vendor SDK library file
install(
  FILES "libfairino/lib/libfairino.so.2.3.5"
  DESTINATION lib/${PROJECT_NAME}
)

# 5. NATIVE AUTOMATION: Direct CMake to generate the required library symlinks inside the install directory
install(CODE "
  execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink 
    \"libfairino.so.2.3.5\" 
    \"\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/libfairino.so.2\")
    
  execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink 
    \"libfairino.so.2.3.5\" 
    \"\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/libfairino.so\")
")


ament_package()
