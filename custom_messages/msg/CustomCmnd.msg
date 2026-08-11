# CustomCmnd.msg
# NOTE(ros2-port): ROS2 rosidl requires snake_case field names; onlineGain1/onlineGain2
# (valid in ROS1) were renamed to online_gain1/online_gain2. Every publisher/subscriber of
# this message was updated to match.

std_msgs/Header header
float64[] position
float64[] velocity
float64[] effort
float64[] online_gain1
float64[] online_gain2

