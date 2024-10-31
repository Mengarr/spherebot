cmake_minimum_required(VERSION 3.8)
project(remote_control_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(i2c_lib REQUIRED)
find_package(control_lib REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)


# List of executables
add_executable(remote_control_node src/remote_control_node.cpp)

# Targets
target_include_directories(remote_control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

# Compile features
target_compile_features(remote_control_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# Dependencies
ament_target_dependencies(remote_control_node 
  rclcpp 
  sensor_msgs
  i2c_lib
  control_lib
  trajectory_msgs 
  control_msgs
)

install(TARGETS remote_control_node
  DESTINATION lib/${PROJECT_NAME})

  install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)
ament_package()
