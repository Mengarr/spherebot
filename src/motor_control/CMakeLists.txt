cmake_minimum_required(VERSION 3.8)
project(motor_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(i2c_lib REQUIRED)
find_package(control_lib REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# List of executables
add_executable(motor_control_node src/motor_control_node.cpp src/motorControl.cpp)

# Targets
target_include_directories(motor_control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

# Compile features
target_compile_features(motor_control_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# Dependencies
ament_target_dependencies(motor_control_node 
  rclcpp 
  i2c_lib
  control_lib
  trajectory_msgs 
  control_msgs
  std_msgs
)

install(TARGETS motor_control_node
  DESTINATION lib/${PROJECT_NAME})


ament_package()
