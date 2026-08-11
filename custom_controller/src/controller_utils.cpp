/*
 Author: Malgorzata Kamedula
 Desc: Effort position controller for the SEA with SEA simulation
 ROS2 port: see controller_utils.h.
*/

#include "custom_controller/controller_utils.h"

#include <chrono>
#include <memory>

using namespace std::chrono_literals;

namespace custom_controller
{

MapToUrdf::MapToUrdf(
  const rclcpp::Node::SharedPtr & node, std::string topic, std::string controller)
{
  RCLCPP_INFO_STREAM(node->get_logger(), "Waiting for callback");

  auto init_sub = node->create_subscription<sensor_msgs::msg::JointState>(
    topic, 1, std::bind(&MapToUrdf::initCallback, this, std::placeholders::_1));

  auto client = node->create_client<custom_services::srv::GetJointNames>(
    controller + "/get_joint_names");

  if (!node->get_parameter(controller + "/dof", dofs_)) {
    RCLCPP_ERROR(
      node->get_logger(), "Number of joints not given 'dof' (namespace: %s)",
      node->get_namespace());
    return;
  }

  id_.reserve(dofs_);

  // NOTE(ros2-port): faithful translation of the ROS1 blocking spin-until-valid
  // loop (rclcpp has no synchronous ros::spinOnce()/single-node equivalent to
  // ros::ok(), so rclcpp::ok() + spin_some replaces it). This class is not
  // wired into any controller (see header note), so this path is never
  // exercised today.
  //
  // The free-function rclcpp::spin_some(node) overload is deprecated -- it
  // builds a fresh SingleThreadedExecutor internally on every call instead of
  // reusing one, so it's replaced here with an explicit executor built once
  // before the loop.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  rclcpp::Rate loop_rate(10.0);
  while (!valid_ && rclcpp::ok()) {
    executor.spin_some();
    loop_rate.sleep();
  }

  auto srv = std::make_shared<custom_services::srv::GetJointNames::Request>();
  auto future = client->async_send_request(srv);
  if (rclcpp::spin_until_future_complete(node, future) == rclcpp::FutureReturnCode::SUCCESS) {
    auto response = future.get();
    for (const auto & joint : response->joints) {
      controller_names_.push_back(joint);
    }

    id_.resize(dofs_, NON_EXISTING);

    for (size_t i = 0; i < controller_names_.size(); i++) {
      for (size_t j = 0; j < urdf_names_.size(); j++) {
        if (controller_names_.at(i) == urdf_names_.at(j)) {
          id_.at(response->ids.at(i)) = static_cast<int>(j);
          break;
        }
      }
    }
  } else {
    RCLCPP_ERROR_STREAM(node->get_logger(), "Server doesn't work");
    id_.resize(dofs_, NON_EXISTING);
  }
}

void MapToUrdf::initCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  for (const auto & name : msg->name) {
    urdf_names_.push_back(name);
  }
  valid_ = true;
}

}  // namespace custom_controller
