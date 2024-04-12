cmake_minimum_required(VERSION 3.5)
project(flir_lepton_auxiliary)

# Find dependencies
find_package(ament_cmake REQUIRED)

# If you have dependencies, they should be found here, e.g., find_package(rclcpp REQUIRED)

# Include directories
# include_directories(include)  # Uncomment if there are specific include directories

# Since this package might only contain auxiliary files like configurations, scripts,
# or data, you might not have libraries or executables to build.
# However, you will specify installation rules for those files.

# Installation of directories and files
# Here you install any directories containing necessary files
# install(DIRECTORY config/
#   DESTINATION share/${PROJECT_NAME}/config
# )

# Install launch files if applicable
# install(DIRECTORY launch/
#   DESTINATION share/${PROJECT_NAME}/launch
# )

# If there are executable scripts, they would be installed like this
# install(PROGRAMS scripts/my_script.py
#   DESTINATION lib/${PROJECT_NAME}
# )

# Export information to downstream packages
ament_package()
