cmake_minimum_required(VERSION 3.8...3.28)
project(custom_services)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/UpdateGains.srv"
  "srv/UpdatePDGains.srv"
  "srv/UpdatePrint.srv"
  "srv/GetJointNames.srv"
  "srv/ComputePayloadEstimate.srv"
  "srv/JointStateCmnd.srv"
  "srv/LoadGains.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_package()
