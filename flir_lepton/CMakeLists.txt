cmake_minimum_required(VERSION 3.5)
project(flir_lepton)

# Find the ament_cmake package
find_package(ament_cmake REQUIRED)

# This is a metapackage
ament_metapackage()

# There are no libraries, binaries, or other files to install for a metapackage
# so no installation directives are needed.

# At the end, we need to call ament_package() to finish the configuration
ament_package()
