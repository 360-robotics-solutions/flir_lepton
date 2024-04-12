cmake_minimum_required(VERSION 3.5)
project(flir_lepton_image_processing)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)  # Uncommented as you might still need it in ROS 2
find_package(flir_lepton_msgs REQUIRED)  # Assuming this package has been ported to ROS 2
find_package(ament_lint_auto REQUIRED)
ament_lint_auto_find_test_dependencies()

# Configure dynamic reconfigure like functionality using parameters in ROS 2
# ROS 2 does not have dynamic_reconfigure in the same way. You'll have to use parameters.

# Include directories
include_directories(
  include/${PROJECT_NAME}
)

# Utils library
add_library(${PROJECT_NAME}_utils
  src/utils/blob_detection.cpp
  src/utils/bounding_box_detection.cpp
  src/utils/edge_detection.cpp
  src/utils/roi_filters.cpp
  src/utils/thermal_rois_conveyor.cpp
  src/utils/message_conversions.cpp
  src/utils/morphological_operators.cpp
  src/utils/outline_discovery.cpp
  src/utils/parameters.cpp
  src/utils/timer.cpp
  src/utils/visualization.cpp
)

# Link libraries
ament_target_dependencies(${PROJECT_NAME}_utils
  rclcpp
  cv_bridge
  image_transport
  flir_lepton_msgs
)

target_link_libraries(${PROJECT_NAME}_utils)

add_subdirectory(src/processing_node)

# ROS 2 does not use roslint, use ament_lint instead for linting
# You may configure ament_lint options via ament_lint_auto

# Install includes, libraries, executables
install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME}_utils
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Export information to downstream packages
ament_package()
