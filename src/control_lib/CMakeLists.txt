cmake_minimum_required(VERSION 3.8)
project(control_lib)

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(i2c_lib REQUIRED)

# Add the library
add_library(control_interfaces
  src/geodeticConverter.cpp
  src/stanleyControl.cpp
  src/auxilary_arduino.cpp
  src/motorControl.cpp
  src/kinematic_transforms.cpp
  src/PID_controller.cpp
  src/LowpassFilter.cpp
)

# Specify include directories for the library
target_include_directories(control_interfaces PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Dependencies
ament_target_dependencies(control_interfaces 
  rclcpp 
  sensor_msgs 
  i2c_lib
)

# Install the library
install(
  TARGETS control_interfaces
  EXPORT export_control_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install the header files
install(
  DIRECTORY include/
  DESTINATION include/
)

# Export the library and dependencies
ament_export_targets(export_control_lib HAS_LIBRARY_TARGET)
ament_export_libraries(control_interfaces)

# Finalize the ament package
ament_package()
