/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  Copyright (c) 2012, hiDOF, Inc.
 *  All rights reserved.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 *********************************************************************/

/*
 Author: Malgorzata Kamedula
 Desc: Effort velocity controller for the SEA with SEA simulation

 ROS2 port note (2026-08-04): 1:1 port of the ROS1 actuator_velocity_controller
 (a genuinely different control law from the position controller -- velocity
 error times a_Kd, no integral/position term -- so it stays a separate plugin
 rather than being folded into ActuatorPositionController).
*/

#ifndef CUSTOM_CONTROLLER__ACTUATOR_VELOCITY_CONTROLLER_H_
#define CUSTOM_CONTROLLER__ACTUATOR_VELOCITY_CONTROLLER_H_

#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "rclcpp/subscription.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "custom_messages/msg/custom_cmnd.hpp"
#include "custom_messages/msg/group_controller_msg.hpp"
#include "custom_services/srv/get_joint_names.hpp"
#include "custom_services/srv/update_gains.hpp"
#include "custom_services/srv/update_print.hpp"

namespace custom_controller
{

enum class VelocityJointType
{
  REVOLUTE,
  CONTINUOUS,
  PRISMATIC,
  UNKNOWN
};

class ActuatorVelocityController : public controller_interface::ControllerInterface
{
public:
  ActuatorVelocityController();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // See ActuatorPositionController::define_custom_node_options() -- same
  // reason (arbitrary dof-sized parameter trees read without pre-declaring
  // every dotted name).
  rclcpp::NodeOptions define_custom_node_options() const override;

  struct Commands
  {
    std::vector<double> position;
    std::vector<double> velocity;
    std::vector<double> effort;
    std::vector<double> online_gain1;
    std::vector<double> online_gain2;
  };

private:
  class Actuator
  {
  public:
    Actuator(double Ks, double D, double B, double phi, double torque_limit)
    : position(0), velocity(0), acceleration(0), motor_torque(0),
      Ks(Ks), D(D), B(B), phi(phi), torque_limit(torque_limit)
    {
    }

    double position;
    double velocity;
    double acceleration;
    double motor_torque;
    double Ks;
    double D;
    double B;
    double phi;
    double torque_limit;

    void update(double period_sec, double commanded_effort);
    void enforceTorqueLimit();
  };

  // Pure velocity-error control law: a_Kd * (desired_velocity - velocity), no
  // integral term -- identical to the ROS1 controller.
  class Controller
  {
  public:
    explicit Controller(double a_Kd)
    : a_Kd(a_Kd), print(false), lower_limit(0), upper_limit(0), type(VelocityJointType::UNKNOWN)
    {
    }

    double a_Kd;
    bool print;
    double lower_limit;
    double upper_limit;
    VelocityJointType type;

    void enforceStateLimits(double & command);
    double update(double desired_velocity, double velocity);
  };

  class Link
  {
  public:
    explicit Link(std::string name)
    : name(std::move(name))
    {
    }

    std::string name;
    double effort = 0;
    hardware_interface::LoanedCommandInterface * command_effort = nullptr;
    hardware_interface::LoanedStateInterface * state_position = nullptr;
    hardware_interface::LoanedStateInterface * state_velocity = nullptr;

    double getPosition() const;
    double getVelocity() const;
    void computeEffort(Actuator & actuator);
    void enforceTorqueLimit(double limit);
    void send();
  };

  bool getDofs();
  bool loadOmittedJoints();
  bool initControllers();
  bool initActuators();
  void initCommand();
  void initDebugPrint();

  bool addPrint(int data, std::string & response);
  bool removePrint(int data, std::string & response);
  std::stringstream printingList();

  void setCommandCallback(const custom_messages::msg::CustomCmnd & msg);
  void setCommand(
    const std::vector<double> & pos_command, const std::vector<double> & vel_command,
    const std::vector<double> & eff_command, const std::vector<double> & gain1_command,
    const std::vector<double> & gain2_command);

  void updateGains(
    const std::shared_ptr<custom_services::srv::UpdateGains::Request> req,
    std::shared_ptr<custom_services::srv::UpdateGains::Response> res);
  void updatePrint(
    const std::shared_ptr<custom_services::srv::UpdatePrint::Request> req,
    std::shared_ptr<custom_services::srv::UpdatePrint::Response> res);
  void getJointNames(
    const std::shared_ptr<custom_services::srv::GetJointNames::Request> req,
    std::shared_ptr<custom_services::srv::GetJointNames::Response> res);
  void setFeedForwardTorque(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
    std::shared_ptr<std_srvs::srv::SetBool::Response> res);

  void publishState(const rclcpp::Time & time);
  void publishDebug(const rclcpp::Time & time);

  int dof_ = 0;
  int raw_dof_ = 0;
  int loop_count_ = 0;
  int update_rate_ = 1;

  std::vector<Link> links_;
  std::vector<Controller> controllers_;
  std::vector<Actuator> actuators_;

  std::vector<int> selector_;
  std::vector<int> reversed_selector_;

  realtime_tools::RealtimeBuffer<Commands> command_buffer_;
  Commands command_struct_;

  rclcpp::Subscription<custom_messages::msg::CustomCmnd>::SharedPtr sub_command_;

  rclcpp::Service<custom_services::srv::UpdateGains>::SharedPtr update_gains_service_;
  rclcpp::Service<custom_services::srv::UpdatePrint>::SharedPtr update_print_service_;
  rclcpp::Service<custom_services::srv::GetJointNames>::SharedPtr get_joint_names_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_ff_torque_service_;

  std::shared_ptr<rclcpp::Publisher<custom_messages::msg::GroupControllerMsg>>
  debug_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<custom_messages::msg::GroupControllerMsg>>
  rt_debug_publisher_;

  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::JointState>> state_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>
  rt_state_publisher_;
};

}  // namespace custom_controller

#endif  // CUSTOM_CONTROLLER__ACTUATOR_VELOCITY_CONTROLLER_H_
