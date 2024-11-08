cmake_minimum_required(VERSION 3.8)
project(joystick_reader)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# List of executables
add_executable(joystick_reader_node src/joystick_reader_node.cpp)

# Targets
target_include_directories(joystick_reader_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

# Compile features
target_compile_features(joystick_reader_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# Dependencies
ament_target_dependencies(joystick_reader_node rclcpp sensor_msgs)

install(TARGETS joystick_reader_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
