cmake_minimum_required(VERSION 3.8)
project(i2c_lib)

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)  # Include any other dependencies as needed

# Add the library
add_library(i2c_interface
  src/i2c_interface.cpp
)

# Specify include directories for the library
target_include_directories(i2c_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link dependencies to the library
ament_target_dependencies(i2c_interface rclcpp)

# Install the library
install(
  TARGETS i2c_interface
  EXPORT export_i2c_lib
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
ament_export_targets(export_i2c_lib HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp)
ament_export_libraries(i2c_interface)

# Finalize the ament package
ament_package()
