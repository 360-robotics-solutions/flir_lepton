cmake_minimum_required(VERSION 3.5)
project(flir_lepton_sensor)

# Ament and ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(flir_lepton_msgs REQUIRED)  # Assuming this package is already ported to ROS 2

find_package(Boost REQUIRED COMPONENTS thread)

# Include directories
include_directories(
  include
)

# Libraries
add_library(${PROJECT_NAME}_utils_lib
  src/flir_lepton_utils.cpp
)

add_library(${PROJECT_NAME}_lib
  src/flir_lepton_hw_iface.cpp
)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  sensor_msgs
  flir_lepton_msgs
)

target_link_libraries(${PROJECT_NAME}_lib
  ${PROJECT_NAME}_utils_lib
)

# Executables
add_executable(${PROJECT_NAME}_node
  src/flir_lepton_sensor.cpp
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  sensor_msgs
  flir_lepton_msgs
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_lib
)

# Install targets
install(TARGETS
  ${PROJECT_NAME}_node
  ${PROJECT_NAME}_lib
  ${PROJECT_NAME}_utils_lib
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# Export dependencies
ament_package()
