cmake_minimum_required(VERSION 3.8)
project(gps_read_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(i2c_lib REQUIRED)
find_package(control_lib REQUIRED)

# List of executables
add_executable(gps_read src/gps_read.cpp src/TinyGPS++.cpp)

# Targets
target_include_directories(gps_read PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

# Compile features
target_compile_features(gps_read PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS gps_read
  DESTINATION lib/${PROJECT_NAME})

# Dependencies
ament_target_dependencies(gps_read rclcpp sensor_msgs i2c_lib control_lib)


install(TARGETS gps_read
  DESTINATION lib/${PROJECT_NAME})


ament_package()
