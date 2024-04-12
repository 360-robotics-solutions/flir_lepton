cmake_minimum_required(VERSION 3.5)
project(flir_lepton_msgs)

# Find ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Specify message files
set(msg_files
  "msg/flir_lepton_sensor/TemperaturesMsg.msg"
  "msg/flir_lepton_sensor/FlirLeptonBatchMsg.msg"
  "msg/flir_lepton_sensor/FlirLeptonRawMsg.msg"
  "msg/flir_lepton_image_processing/CandidateRoiMsg.msg"
  "msg/flir_lepton_image_processing/CandidateRoisVectorMsg.msg"
  "msg/flir_lepton_image_processing/GeneralAlertInfo.msg"
  "msg/flir_lepton_image_processing/ThermalAlert.msg"
  "msg/flir_lepton_image_processing/ThermalAlertVector.msg"
)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs sensor_msgs
)

# Ament package setup
ament_export_dependencies(rosidl_default_runtime std_msgs sensor_msgs)
ament_package()
