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
 ROS2 port: see actuator_velocity_controller.h.
*/

#include "custom_controller/actuator_velocity_controller.h"

#include <algorithm>
#include <limits>
#include <numeric>
#include <utility>

#include "pluginlib/class_list_macros.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

namespace custom_controller
{

ActuatorVelocityController::ActuatorVelocityController()
: controller_interface::ControllerInterface()
{
}

controller_interface::InterfaceConfiguration
ActuatorVelocityController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & link : links_) {
    config.names.push_back(link.name + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
ActuatorVelocityController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & link : links_) {
    config.names.push_back(link.name + "/position");
    config.names.push_back(link.name + "/velocity");
  }
  return config;
}

controller_interface::CallbackReturn ActuatorVelocityController::on_init()
{
  return controller_interface::CallbackReturn::SUCCESS;
}

rclcpp::NodeOptions ActuatorVelocityController::define_custom_node_options() const
{
  rclcpp::NodeOptions node_options = ControllerInterfaceBase::define_custom_node_options();
  node_options.automatically_declare_parameters_from_overrides(true);
  return node_options;
}

bool ActuatorVelocityController::getDofs()
{
  auto node = get_node();
  int64_t dof_param = 0;
  if (!node->get_parameter("dof", dof_param) || dof_param <= 0) {
    RCLCPP_ERROR(
      node->get_logger(),
      "Couldn't find number of joints defined for this controller. Please define parameter "
      "'dof'.");
    return false;
  }

  raw_dof_ = static_cast<int>(dof_param);
  dof_ = raw_dof_;

  selector_.resize(raw_dof_);
  std::iota(selector_.begin(), selector_.end(), 0);

  reversed_selector_.resize(raw_dof_, -1);
  return true;
}

bool ActuatorVelocityController::loadOmittedJoints()
{
  auto node = get_node();
  std::vector<int64_t> omit_list;
  if (!node->get_parameter_or("omit", omit_list, std::vector<int64_t>{}) || omit_list.empty()) {
    return true;
  }

  std::stringstream str;
  str << "Omits following joints " << node->get_name() << std::endl;

  for (const auto & raw_omit : omit_list) {
    int omit = static_cast<int>(raw_omit) - 1;

    auto exists = std::find(selector_.begin(), selector_.end(), omit);
    if (exists == selector_.end()) {
      continue;
    }

    selector_.erase(exists);
    str << "joint" << omit + 1 << std::endl;
  }

  dof_ = static_cast<int>(selector_.size());

  for (size_t i = 0; i < selector_.size(); ++i) {
    reversed_selector_.at(selector_[i]) = static_cast<int>(i);
  }

  RCLCPP_INFO_STREAM(node->get_logger(), str.str());
  return true;
}

bool ActuatorVelocityController::initControllers()
{
  auto node = get_node();
  const auto & hard_limits = get_hard_joint_limits();

  for (const auto & i : selector_) {
    const std::string prefix = "joint" + std::to_string(i + 1) + ".";

    std::string name;
    double a_Kd;
    if (!node->get_parameter(prefix + "name", name) || !node->get_parameter(prefix + "a_Kd", a_Kd))
    {
      RCLCPP_ERROR_STREAM(
        node->get_logger(), "Couldn't find a complete description of " << prefix);
      return false;
    }

    controllers_.push_back(Controller(a_Kd));
    links_.push_back(Link(name));

    auto it = hard_limits.find(name);
    if (it != hard_limits.end() && it->second.has_velocity_limits) {
      controllers_.back().lower_limit = -it->second.max_velocity;
      controllers_.back().upper_limit = it->second.max_velocity;
      controllers_.back().type = VelocityJointType::REVOLUTE;
    } else {
      controllers_.back().type = VelocityJointType::CONTINUOUS;
    }
  }

  return true;
}

bool ActuatorVelocityController::initActuators()
{
  auto node = get_node();

  for (size_t i = 0; i < selector_.size(); ++i) {
    const std::string & name = links_.at(i).name;
    const std::string prefix = "actuators." + name + ".";

    double Ks, D, phi, torque_limit;
    if (
      !node->get_parameter(prefix + "Ks", Ks) ||
      !node->get_parameter(prefix + "D", D) ||
      !node->get_parameter(prefix + "phim", phi) ||
      !node->get_parameter(prefix + "Torque_limit", torque_limit))
    {
      RCLCPP_ERROR_STREAM(node->get_logger(), "Couldn't find actuators data for joint " << name);
      return false;
    }

    int64_t nu = 1;
    node->get_parameter_or(prefix + "nu", nu, static_cast<int64_t>(1));

    double B;
    if (node->has_parameter(prefix + "Bm")) {
      node->get_parameter(prefix + "Bm", B);
      B = B * static_cast<double>(nu) * static_cast<double>(nu);
    } else if (node->has_parameter(prefix + "B")) {
      node->get_parameter(prefix + "B", B);
    } else {
      RCLCPP_ERROR_STREAM(node->get_logger(), "Motor inertia for " << name << " undefined.");
      return false;
    }

    actuators_.push_back(
      Actuator(Ks, D, B, phi * static_cast<double>(nu) * static_cast<double>(nu), torque_limit));
  }

  return true;
}

void ActuatorVelocityController::initCommand()
{
  command_struct_.position.resize(raw_dof_, 0);
  command_struct_.velocity.resize(raw_dof_, 0);
  command_struct_.effort.resize(raw_dof_, 0);
  command_struct_.online_gain1.resize(raw_dof_, 0);
  command_struct_.online_gain2.resize(raw_dof_, 0);
}

void ActuatorVelocityController::initDebugPrint()
{
  auto node = get_node();
  std::vector<int64_t> print_list;
  if (!node->get_parameter_or("print_data", print_list, std::vector<int64_t>{})) {
    return;
  }

  std::string message;
  for (const auto & raw_data : print_list) {
    int data = static_cast<int>(raw_data);
    const std::string prefix = "joint" + std::to_string(data) + ".";
    std::string name;
    if (!node->get_parameter(prefix + "name", name)) {
      RCLCPP_WARN_STREAM(
        node->get_logger(), "Couldn't retrieve name of joint " << data << " from parameters");
      continue;
    }

    auto number = std::find_if(
      links_.begin(), links_.end(), [&name](const Link & link) {return link.name == name;});
    int nr = static_cast<int>(std::distance(links_.begin(), number));

    addPrint(nr, message);
  }

  RCLCPP_INFO_STREAM(node->get_logger(), message);
}

controller_interface::CallbackReturn ActuatorVelocityController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto node = get_node();

  if (!getDofs()) {
    return controller_interface::CallbackReturn::ERROR;
  }
  initCommand();

  if (!loadOmittedJoints()) {
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!initControllers()) {
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!initActuators()) {
    return controller_interface::CallbackReturn::ERROR;
  }

  initDebugPrint();

  int64_t update_rate = 1;
  node->get_parameter_or("controller_update", update_rate, static_cast<int64_t>(1));
  update_rate_ = static_cast<int>(update_rate);

  // NOTE(ros2-port fix, 2026-08-06): see the matching note in
  // actuator_position_controller.cpp -- relative names resolve against the
  // node's namespace only, private ("~/...") names resolve against the
  // fully-qualified node name, which is what gives the ROS1-equivalent
  // /velocity_controller/... (well, /<controller_name>/...) scoping.
  debug_publisher_ = get_node()->create_publisher<custom_messages::msg::GroupControllerMsg>(
    "~/debug", rclcpp::SystemDefaultsQoS());
  rt_debug_publisher_ =
    std::make_unique<realtime_tools::RealtimePublisher<custom_messages::msg::GroupControllerMsg>>(
    debug_publisher_);

  state_publisher_ = get_node()->create_publisher<sensor_msgs::msg::JointState>(
    "~/state", rclcpp::SystemDefaultsQoS());
  rt_state_publisher_ =
    std::make_unique<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>(
    state_publisher_);

  sub_command_ = get_node()->create_subscription<custom_messages::msg::CustomCmnd>(
    "~/command", rclcpp::SystemDefaultsQoS(),
    std::bind(&ActuatorVelocityController::setCommandCallback, this, _1));

  update_gains_service_ = get_node()->create_service<custom_services::srv::UpdateGains>(
    "~/update_gains", std::bind(&ActuatorVelocityController::updateGains, this, _1, _2));
  update_print_service_ = get_node()->create_service<custom_services::srv::UpdatePrint>(
    "~/update_print", std::bind(&ActuatorVelocityController::updatePrint, this, _1, _2));
  get_joint_names_service_ = get_node()->create_service<custom_services::srv::GetJointNames>(
    "~/get_joint_names", std::bind(&ActuatorVelocityController::getJointNames, this, _1, _2));
  set_ff_torque_service_ = get_node()->create_service<std_srvs::srv::SetBool>(
    "~/set_ff_torque", std::bind(&ActuatorVelocityController::setFeedForwardTorque, this, _1, _2));

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ActuatorVelocityController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (auto & link : links_) {
    for (auto & command_interface : command_interfaces_) {
      if (command_interface.get_name() == link.name + "/effort") {
        link.command_effort = &command_interface;
      }
    }
    for (auto & state_interface : state_interfaces_) {
      if (state_interface.get_name() == link.name + "/position") {
        link.state_position = &state_interface;
      } else if (state_interface.get_name() == link.name + "/velocity") {
        link.state_velocity = &state_interface;
      }
    }
  }

  for (int i = 0; i < dof_; ++i) {
    actuators_.at(i).position = links_.at(i).getPosition();
    actuators_.at(i).velocity = links_.at(i).getVelocity();
  }
  command_buffer_.initRT(command_struct_);
  loop_count_ = 0;

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ActuatorVelocityController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (auto & link : links_) {
    link.command_effort = nullptr;
    link.state_position = nullptr;
    link.state_velocity = nullptr;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type ActuatorVelocityController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  command_struct_ = *(command_buffer_.readFromRT());
  double period_sec = period.seconds();
  bool do_update = !(loop_count_ % update_rate_);

  for (int i = 0; i < dof_; ++i) {
    int real_id = selector_.at(i);
    double command_velocity = command_struct_.velocity[real_id];

    if (do_update) {
      actuators_.at(i).motor_torque =
        controllers_.at(i).update(command_velocity, actuators_.at(i).velocity);
    }

    actuators_.at(i).enforceTorqueLimit();
    links_.at(i).computeEffort(actuators_.at(i));
    actuators_.at(i).update(period_sec, links_.at(i).effort);
  }

  for (int i = 0; i < dof_; ++i) {
    links_.at(i).send();
  }

  if (do_update) {
    publishState(time);
    publishDebug(time);
  }

  loop_count_++;
  return controller_interface::return_type::OK;
}

double ActuatorVelocityController::Link::getPosition() const
{
  if (state_position == nullptr) {
    return 0.0;
  }
  return state_position->get_optional<double>().value_or(0.0);
}

double ActuatorVelocityController::Link::getVelocity() const
{
  if (state_velocity == nullptr) {
    return 0.0;
  }
  return state_velocity->get_optional<double>().value_or(0.0);
}

void ActuatorVelocityController::Link::send()
{
  if (command_effort != nullptr) {
    static_cast<void>(command_effort->set_value<double>(effort));
  }
}

void ActuatorVelocityController::Link::enforceTorqueLimit(double limit)
{
  effort = (std::fabs(effort) < limit) ? effort : effort / std::fabs(effort) * limit;
}

void ActuatorVelocityController::Link::computeEffort(Actuator & actuator)
{
  effort = actuator.Ks * (actuator.position - getPosition()) +
    actuator.D * (actuator.velocity - getVelocity());
  enforceTorqueLimit(actuator.torque_limit);
}

double ActuatorVelocityController::Controller::update(double desired_velocity, double velocity)
{
  enforceStateLimits(desired_velocity);
  double error = desired_velocity - velocity;
  return a_Kd * error;
}

void ActuatorVelocityController::Controller::enforceStateLimits(double & command)
{
  if (command > upper_limit) {
    command = upper_limit;
  } else if (command < lower_limit) {
    command = lower_limit;
  }
}

void ActuatorVelocityController::Actuator::enforceTorqueLimit()
{
  motor_torque = (std::fabs(motor_torque) < torque_limit) ?
    motor_torque :
    motor_torque / std::fabs(motor_torque) * torque_limit;
}

void ActuatorVelocityController::Actuator::update(double period_sec, double commanded_effort)
{
  acceleration = (motor_torque - phi * velocity - commanded_effort) / B;
  position += 0.5 * acceleration * period_sec * period_sec + velocity * period_sec;
  velocity += acceleration * period_sec;
}

void ActuatorVelocityController::setCommand(
  const std::vector<double> & pos_command, const std::vector<double> & vel_command,
  const std::vector<double> & eff_command, const std::vector<double> & gain1_command,
  const std::vector<double> & gain2_command)
{
  command_struct_.position = pos_command;
  command_struct_.velocity = vel_command;
  command_struct_.effort = eff_command;
  command_struct_.online_gain1 = gain1_command;
  command_struct_.online_gain2 = gain2_command;
  command_buffer_.writeFromNonRT(command_struct_);
}

void ActuatorVelocityController::setCommandCallback(const custom_messages::msg::CustomCmnd & msg)
{
  setCommand(msg.position, msg.velocity, msg.effort, msg.online_gain1, msg.online_gain2);
}

void ActuatorVelocityController::publishState(const rclcpp::Time & time)
{
  if (!rt_state_publisher_) {
    return;
  }

  sensor_msgs::msg::JointState msg;
  for (int i = 0; i < dof_; ++i) {
    msg.position.push_back(actuators_.at(i).position);
    msg.velocity.push_back(actuators_.at(i).velocity);
    msg.effort.push_back(actuators_.at(i).motor_torque);
  }
  msg.header.stamp = time;
  rt_state_publisher_->try_publish(msg);
}

void ActuatorVelocityController::publishDebug(const rclcpp::Time & time)
{
  if (!rt_debug_publisher_) {
    return;
  }

  custom_messages::msg::GroupControllerMsg msg;
  command_struct_ = *(command_buffer_.readFromRT());

  for (int i = 0; i < dof_; ++i) {
    if (!controllers_.at(i).print) {
      continue;
    }

    custom_messages::msg::ControllerMsg pub;
    pub.link_side_position = links_.at(i).getPosition();
    pub.link_side_velocity = links_.at(i).getVelocity();
    pub.link_side_effort = links_.at(i).effort;
    pub.motor_side_position = actuators_.at(i).position;
    pub.motor_side_velocity = actuators_.at(i).velocity;
    pub.motor_side_effort = actuators_.at(i).motor_torque;
    pub.desired_position = command_struct_.position[selector_.at(i)];
    msg.controller.push_back(pub);
  }

  msg.header.stamp = time;
  rt_debug_publisher_->try_publish(msg);
}

std::stringstream ActuatorVelocityController::printingList()
{
  std::stringstream msg;
  msg << "\n List of joints in controller publisher\n";
  int k = 0;
  for (size_t i = 0; i < controllers_.size(); ++i) {
    if (!controllers_.at(i).print) {
      continue;
    }
    msg << "printing number: " << k << "\t joint number: " << i << "\t joint name: " <<
      links_.at(i).name << "." << std::endl;
    k++;
  }
  return msg;
}

bool ActuatorVelocityController::addPrint(int data, std::string & response)
{
  std::stringstream str;
  if (data < 0 || data > dof_ - 1) {
    str << "Received number: " << data <<
      " is not defined within the controller. Please send a number between 0 and " <<
      dof_ - 1 << ".\n" << "To see loaded joints call 'get_joint_names' service." << std::endl;
    response += str.str();
    return false;
  }

  if (controllers_.at(data).print) {
    str << "Requested joint nr " << data << " is already printed by the controller" << std::endl;
    response += str.str();
    return false;
  }

  controllers_.at(data).print = true;
  str << "joint " << data << ", joint_name: " << links_.at(data).name << std::endl;
  response += str.str();
  return true;
}

bool ActuatorVelocityController::removePrint(int data, std::string & response)
{
  std::stringstream str;
  if (data < 0 || data > dof_ - 1) {
    str << "Received number: " << data <<
      " is not defined within the controller. Please send a number between 0 and " <<
      dof_ - 1 << ".\n" << "To see loaded joints call 'get_joint_names' service." << std::endl;
    response += str.str();
    return false;
  }
  if (!controllers_.at(data).print) {
    str << "Requested joint " << data << " has not been printed by the controller" << std::endl;
    response += str.str();
    return false;
  }

  controllers_.at(data).print = false;
  response += str.str();
  return true;
}

void ActuatorVelocityController::updateGains(
  const std::shared_ptr<custom_services::srv::UpdateGains::Request> req,
  std::shared_ptr<custom_services::srv::UpdateGains::Response> res)
{
  std::stringstream msg;

  // Unlike the position controller, the ROS1 velocity controller checked
  // req.nr directly (not through reversed_selector) before indexing -- no
  // out-of-bounds read here, preserved as-is.
  if (req->nr < 0 || req->nr > dof_ - 1) {
    res->success = false;
    msg << "Received number: " << req->nr <<
      " is not defined within the controller. Please send a number between 0 and " <<
      dof_ - 1 << "." << std::endl << "To see loaded joints call 'get_joint_names' service.";
    res->message = msg.str();
    return;
  }

  res->success = true;

  if (req->a_p != 0) {
    msg << "Actuator proportional gain is ignored in this controller." << std::endl;
    res->success = false;
  }

  if (req->a_d >= 0) {
    controllers_.at(req->nr).a_Kd = req->a_d;
  } else {
    msg << "Given actuator derivative gain is negative. Please give a positive number." <<
      std::endl << "Actuator derivative gain not changed." << std::endl;
    res->success = false;
  }

  if (req->j_p != 0) {
    msg << "Joint proportional gain is ignored in this controller." << std::endl;
    res->success = false;
  }

  if (req->j_d != 0) {
    msg << "Joint derivative gain is ignored in this controller." << std::endl;
    res->success = false;
  }
  if (req->i >= 0) {
    msg << "Joint integral gain is ignored in this controller." << std::endl;
    res->success = false;
  }

  msg << "Changed controller gains for" << std::endl << "joint number: " << req->nr <<
    std::endl << "joint name: " << links_.at(req->nr).name << "." << std::endl <<
    "Current gains are: " << std::endl << "a_Kp: ignored " << std::endl << "a_Kd: " <<
    controllers_.at(req->nr).a_Kd << std::endl << "j_Kp: ignored " << std::endl <<
    "j_Kd: ignored " << std::endl << "Ki: ignored" << std::endl;
  res->message = msg.str();
}

void ActuatorVelocityController::updatePrint(
  const std::shared_ptr<custom_services::srv::UpdatePrint::Request> req,
  std::shared_ptr<custom_services::srv::UpdatePrint::Response> res)
{
  if (req->command == "add") {
    res->message = "Added following joints of controller publisher\n";
    res->success = addPrint(req->nr, res->message);
  } else if (req->command == "remove") {
    res->success = removePrint(req->nr, res->message);
    res->message += printingList().str();
    res->success = true;
  } else if (req->command == "check") {
    res->message += "\n List of joints in controller publisher\n";
    res->message += printingList().str();

    for (size_t i = 0; i < selector_.size(); ++i) {
      if (controllers_.at(i).print) {
        res->ids.push_back(selector_.at(i));
      }
    }
    res->success = true;
  } else {
    res->message = "unknown command value, send 'add' to publish joint states, "
      "'remove' to stop publishing joint data, and 'check' to examine current state";
    res->success = false;
  }
}

void ActuatorVelocityController::getJointNames(
  const std::shared_ptr<custom_services::srv::GetJointNames::Request>/*req*/,
  std::shared_ptr<custom_services::srv::GetJointNames::Response> res)
{
  res->success = true;
  std::stringstream msg;
  msg << "joint number: \t joint name" << std::endl;

  for (int j = 0; j < dof_; ++j) {
    msg << selector_[j] << "\t" << links_.at(j).name << std::endl;
    res->joints.push_back(links_.at(j).name);
    res->ids.push_back(selector_[j]);
  }
  res->message = msg.str();
}

void ActuatorVelocityController::setFeedForwardTorque(
  const std::shared_ptr<std_srvs::srv::SetBool::Request>/*req*/,
  std::shared_ptr<std_srvs::srv::SetBool::Response> res)
{
  res->success = true;
  res->message = "Feedforward torque term is not supported for this controller.";
}

}  // namespace custom_controller

PLUGINLIB_EXPORT_CLASS(
  custom_controller::ActuatorVelocityController, controller_interface::ControllerInterface)
