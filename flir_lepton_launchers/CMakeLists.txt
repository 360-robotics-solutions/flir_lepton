cmake_minimum_required(VERSION 3.5)
project(flir_lepton_launchers)

# Find the ament_cmake package, which is required for building ROS 2 packages
find_package(ament_cmake REQUIRED)

# Since this is likely a launch-only package with no libraries or binaries,
# we don't need to declare libraries or executables, unless explicitly required.

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# At the end, we need to call ament_package() to finish the configuration
ament_package()
