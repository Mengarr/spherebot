cmake_minimum_required(VERSION 3.8)
project(dfr_10_dof_pkg)

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(i2c_lib REQUIRED)
find_package(control_lib REQUIRED)

# Declare C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# imu node
add_executable(imu_data_node 
  src/imu_data_node.cpp 
  src/FIMU_ITG3200.cpp 
  src/FIMU_ADXL345.cpp 
)
ament_target_dependencies(imu_data_node 
  rclcpp 
  sensor_msgs
  i2c_lib  # Ensure i2c_lib is a dependency
)

# magnometer calibration node
add_executable(calibrate_magnetometer_node
  src/calibrate_magnetometer_node.cpp
  src/VCM5883L.cpp
)
ament_target_dependencies(calibrate_magnetometer_node 
  rclcpp 
  i2c_lib  # Ensure i2c_lib is a dependency
)

# magnometer node
add_executable(magneometer_node
  src/magnetometer_data_node.cpp
  src/VCM5883L.cpp
)
ament_target_dependencies(magneometer_node 
  rclcpp 
  std_msgs
  sensor_msgs
  i2c_lib  # Ensure i2c_lib is a dependency
  control_lib
)

# environmental data node
add_executable(environmental_node
  src/environmental_data_node.cpp
  src/BMP280.cpp
)
ament_target_dependencies(environmental_node 
  rclcpp 
  std_msgs
  i2c_lib  # Ensure i2c_lib is a dependency
)

# Specify include directories for your package's own headers
target_include_directories(imu_data_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

# Specify include directories for your package's own headers
target_include_directories(magneometer_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

# Specify include directories for your package's own headers
target_include_directories(calibrate_magnetometer_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

# Specify include directories for your package's own headers
target_include_directories(environmental_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

# Install the executable
install(TARGETS 
  imu_data_node
  magneometer_node
  environmental_node
  calibrate_magnetometer_node
  DESTINATION lib/${PROJECT_NAME}
)

# Finalize the ament package
ament_package()
