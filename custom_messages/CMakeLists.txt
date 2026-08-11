cmake_minimum_required(VERSION 3.8...3.28)
project(custom_messages)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# GroupJointControllerStateMsg.msg was dropped: it embedded control_msgs/JointControllerState,
# a ROS1-only type with no ROS2 equivalent, and nothing in this workspace referenced it.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CommandMsg.msg"
  "msg/ControllerMsg.msg"
  "msg/CustomCmnd.msg"
  "msg/GroupControllerMsg.msg"
  "msg/StateMsg.msg"
  DEPENDENCIES std_msgs
)

ament_package()
