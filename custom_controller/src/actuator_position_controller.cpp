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
 Desc: Effort position controller for the SEA with SEA simulation
 ROS2 port: see actuator_position_controller.h for the consolidation note.
*/

#include "custom_controller/actuator_position_controller.h"

#include <algorithm>
#include <limits>
#include <numeric>
#include <utility>

#include "pluginlib/class_list_macros.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

namespace custom_controller
{

ActuatorPositionController::ActuatorPositionController()
: controller_interface::ControllerInterface()
{
}

controller_interface::InterfaceConfiguration
ActuatorPositionController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & link : links_) {
    config.names.push_back(link.name + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
ActuatorPositionController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & link : links_) {
    config.names.push_back(link.name + "/position");
    config.names.push_back(link.name + "/velocity");
  }
  return config;
}

controller_interface::CallbackReturn ActuatorPositionController::on_init()
{
  return controller_interface::CallbackReturn::SUCCESS;
}

rclcpp::NodeOptions ActuatorPositionController::define_custom_node_options() const
{
  rclcpp::NodeOptions node_options = ControllerInterfaceBase::define_custom_node_options();
  node_options.automatically_declare_parameters_from_overrides(true);
  return node_options;
}

bool ActuatorPositionController::getDofs()
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

bool ActuatorPositionController::loadOmittedJoints()
{
  auto node = get_node();
  std::vector<int64_t> omit_list;
  if (!node->get_parameter_or("omit", omit_list, std::vector<int64_t>{}) || omit_list.empty()) {
    // NOTE(ros2-port): faithfully preserved from the ROS1 "_classes" controller --
    // when no "omit" parameter is configured (the default, common case), execution
    // returns here WITHOUT ever populating reversed_selector_, so every entry stays
    // -1. That means the updateGains service will report every joint as "not
    // defined" until a (possibly empty) "omit" list is explicitly set. See code
    // review 2026-08-04, custom_controller_classes.cpp:_loadedDofs.
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

bool ActuatorPositionController::initControllers()
{
  auto node = get_node();
  const auto & hard_limits = get_hard_joint_limits();

  for (const auto & i : selector_) {
    const std::string prefix = "joint" + std::to_string(i + 1) + ".";

    std::string name;
    double a_Kp, a_Kd, j_Kp, j_Kd, Ki;
    if (
      !node->get_parameter(prefix + "name", name) ||
      !node->get_parameter(prefix + "a_Kp", a_Kp) ||
      !node->get_parameter(prefix + "a_Kd", a_Kd) ||
      !node->get_parameter(prefix + "j_Kp", j_Kp) ||
      !node->get_parameter(prefix + "j_Kd", j_Kd) ||
      !node->get_parameter(prefix + "Ki", Ki))
    {
      RCLCPP_ERROR_STREAM(
        node->get_logger(), "Couldn't find a complete description of " << prefix);
      return false;
    }

    controllers_.push_back(Controller(a_Kp, a_Kd, j_Kp, j_Kd, Ki));
    links_.push_back(Link(name));

    auto it = hard_limits.find(name);
    if (it != hard_limits.end() && it->second.has_position_limits) {
      controllers_.back().lower_limit = it->second.min_position;
      controllers_.back().upper_limit = it->second.max_position;
      controllers_.back().type = JointType::REVOLUTE;
    } else {
      // NOTE(ros2-port): ros2_control's joint_limits map doesn't carry the URDF
      // joint TYPE the way ROS1's urdf::Joint did -- a joint with no position
      // limits is treated as CONTINUOUS (matching URDF semantics), which is the
      // only distinction enforceStateLimits() actually uses.
      controllers_.back().type = JointType::CONTINUOUS;
    }
  }

  return true;
}

bool ActuatorPositionController::initActuators()
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

void ActuatorPositionController::initCommand()
{
  // Commanded position until the first message on ~/command (2026-10-12):
  // with 0 the PANDORA legs start fully retracted and the leg linkage rests
  // on terrain crests, so an initial raised posture can be configured.
  double initial_position = 0.0;
  get_node()->get_parameter_or("initial_position", initial_position, 0.0);
  command_struct_.position.resize(raw_dof_, initial_position);
  command_struct_.velocity.resize(raw_dof_, 0);
  command_struct_.effort.resize(raw_dof_, 0);
  command_struct_.online_gain1.resize(raw_dof_, 0);
  command_struct_.online_gain2.resize(raw_dof_, 0);
}

void ActuatorPositionController::initDebugPrint()
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

controller_interface::CallbackReturn ActuatorPositionController::on_configure(
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

  node->get_parameter_or("integral_on_link", integral_on_link_, true);
  node->get_parameter_or("startup_rate", startup_rate_, 0.0);

  // NOTE(ros2-port fix, 2026-08-06): relative names ("debug", "command", ...)
  // resolve against the node's *namespace* only in ROS2, not its name -- on
  // controller_manager's default (root) namespace that produced /debug,
  // /command etc. instead of /position_controller/debug,
  // /position_controller/command. Private names ("~/...") resolve against
  // the fully-qualified node name instead, giving the ROS1-equivalent
  // per-controller scoping every other node in this workspace (h_control.py,
  // ik_server.py, world_broadcaster.py) already assumes.
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
    std::bind(&ActuatorPositionController::setCommandCallback, this, _1));

  update_gains_service_ = get_node()->create_service<custom_services::srv::UpdateGains>(
    "~/update_gains", std::bind(&ActuatorPositionController::updateGains, this, _1, _2));
  update_print_service_ = get_node()->create_service<custom_services::srv::UpdatePrint>(
    "~/update_print", std::bind(&ActuatorPositionController::updatePrint, this, _1, _2));
  get_joint_names_service_ = get_node()->create_service<custom_services::srv::GetJointNames>(
    "~/get_joint_names", std::bind(&ActuatorPositionController::getJointNames, this, _1, _2));
  set_ff_torque_service_ = get_node()->create_service<std_srvs::srv::SetBool>(
    "~/set_ff_torque", std::bind(&ActuatorPositionController::setFeedForwardTorque, this, _1, _2));

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ActuatorPositionController::on_activate(
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
    controllers_.at(i).integrator_error = 0.0;
  }
  command_buffer_.initRT(command_struct_);
  loop_count_ = 0;

  // Soft start: begin at the measured link positions (see startup_rate_).
  command_received_ = false;
  startup_active_ = startup_rate_ > 0.0;
  startup_position_.assign(raw_dof_, 0.0);
  for (int i = 0; i < dof_; ++i) {
    startup_position_.at(selector_.at(i)) = links_.at(i).getPosition();
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ActuatorPositionController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (auto & link : links_) {
    link.command_effort = nullptr;
    link.state_position = nullptr;
    link.state_velocity = nullptr;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type ActuatorPositionController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  command_struct_ = *(command_buffer_.readFromRT());
  double period_sec = period.seconds();
  bool do_update = !(loop_count_ % update_rate_);

  if (startup_active_ && command_received_) {
    startup_active_ = false;
  }
  bool startup_done = true;

  for (int i = 0; i < dof_; ++i) {
    int real_id = selector_.at(i);

    double command_position = command_struct_.position[real_id];
    if (startup_active_) {
      // slew from the measured position towards initial_position
      double max_step = startup_rate_ * period_sec;
      double & p = startup_position_.at(real_id);
      p += std::max(-max_step, std::min(max_step, command_position - p));
      if (p != command_position) {
        startup_done = false;
      }
      command_position = p;
    }
    double command_effort = command_struct_.effort[real_id];

    if (!ff_torque_enabled_) {
      command_effort = 0;
    }

    if (do_update) {
      // the control law runs once every update_rate_ loops, so its sample
      // time (for the integral) is update_rate_ * period.
      actuators_.at(i).motor_torque = controllers_.at(i).update(
        command_position, command_effort, actuators_.at(i).position, actuators_.at(i).velocity,
        links_.at(i).getPosition(), links_.at(i).getVelocity(),
        period_sec * update_rate_, actuators_.at(i).torque_limit, integral_on_link_);
    }

    actuators_.at(i).enforceTorqueLimit();
    links_.at(i).computeEffort(actuators_.at(i));
    actuators_.at(i).update(period_sec, links_.at(i).effort);
  }

  for (int i = 0; i < dof_; ++i) {
    links_.at(i).send();
  }

  if (startup_active_ && startup_done) {
    startup_active_ = false;
  }

  if (do_update) {
    publishState(time);
    publishDebug(time);
  }

  loop_count_++;
  return controller_interface::return_type::OK;
}

double ActuatorPositionController::Link::getPosition() const
{
  if (state_position == nullptr) {
    return 0.0;
  }
  return state_position->get_optional<double>().value_or(0.0);
}

double ActuatorPositionController::Link::getVelocity() const
{
  if (state_velocity == nullptr) {
    return 0.0;
  }
  return state_velocity->get_optional<double>().value_or(0.0);
}

void ActuatorPositionController::Link::send()
{
  if (command_effort != nullptr) {
    static_cast<void>(command_effort->set_value<double>(effort));
  }
}

void ActuatorPositionController::Link::enforceTorqueLimit(double limit)
{
  effort = (std::fabs(effort) < limit) ? effort : effort / std::fabs(effort) * limit;
}

void ActuatorPositionController::Link::computeEffort(Actuator & actuator)
{
  effort = actuator.Ks * (actuator.position - getPosition()) +
    actuator.D * (actuator.velocity - getVelocity());
  enforceTorqueLimit(actuator.torque_limit);
}

double ActuatorPositionController::Controller::update(
  double desired_position, double feed_forward, double motor_position,
  double motor_velocity, double link_position, double link_velocity,
  double dt, double integral_limit, bool integral_on_link)
{
  enforceStateLimits(desired_position);
  double error = desired_position - motor_position;
  double integrated_error = integral_on_link ? desired_position - link_position : error;

  // Fix (2026-10-12): was `integrator_error += error` (per-sample sum, no dt,
  // no limit) -- Ki then depended on update_rate and wound up without bound
  // whenever the torque saturated (e.g. a wheel stuck against an obstacle).
  integrator_error += integrated_error * dt;
  if (Ki > 0.0) {
    const double i_max = integral_limit / Ki;
    integrator_error = std::clamp(integrator_error, -i_max, i_max);
  }

  return feed_forward + Ki * integrator_error + a_Kp * error - a_Kd * motor_velocity -
         j_Kd * link_velocity;
}

void ActuatorPositionController::Controller::enforceStateLimits(double & command)
{
  if (type == JointType::CONTINUOUS) {
    return;
  }
  if (command > upper_limit) {
    command = upper_limit;
  } else if (command < lower_limit) {
    command = lower_limit;
  }
}

void ActuatorPositionController::Actuator::enforceTorqueLimit()
{
  motor_torque = (std::fabs(motor_torque) < torque_limit) ?
    motor_torque :
    motor_torque / std::fabs(motor_torque) * torque_limit;
}

void ActuatorPositionController::Actuator::update(double period_sec, double commanded_effort)
{
  acceleration = (motor_torque - phi * velocity - commanded_effort) / B;
  position += 0.5 * acceleration * period_sec * period_sec + velocity * period_sec;
  velocity += acceleration * period_sec;
}

void ActuatorPositionController::setCommand(
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
  command_received_ = true;
}

void ActuatorPositionController::setCommandCallback(const custom_messages::msg::CustomCmnd & msg)
{
  setCommand(msg.position, msg.velocity, msg.effort, msg.online_gain1, msg.online_gain2);
}

void ActuatorPositionController::publishState(const rclcpp::Time & time)
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

void ActuatorPositionController::publishDebug(const rclcpp::Time & time)
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

std::stringstream ActuatorPositionController::printingList()
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

bool ActuatorPositionController::addPrint(int data, std::string & response)
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

bool ActuatorPositionController::removePrint(int data, std::string & response)
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

void ActuatorPositionController::updateGains(
  const std::shared_ptr<custom_services::srv::UpdateGains::Request> req,
  std::shared_ptr<custom_services::srv::UpdateGains::Response> res)
{
  std::stringstream msg;

  // NOTE(ros2-port): faithfully preserved from the ROS1 "_classes" controller --
  // req->nr indexes reversed_selector_ via operator[] BEFORE any bounds check on
  // req->nr itself (only the resulting *value* is range-checked below). A
  // request with an out-of-range nr is an out-of-bounds read. See code review
  // 2026-08-04, actuator_position_controller_classes.cpp:updateGains.
  int called_dof = reversed_selector_[req->nr];
  if (called_dof < 0 || called_dof > dof_ - 1) {
    res->success = false;
    msg << "Received number: " << req->nr <<
      " is not defined within the controller. To see loaded joints call 'get_joint_names' "
      "service.";
    res->message = msg.str();
    return;
  }

  res->success = true;

  if (req->a_p >= 0) {
    controllers_.at(called_dof).a_Kp = req->a_p;
  } else {
    msg << "Given actuator proportional gain is negative. Please give a positive number." <<
      std::endl << "Actuator proportional gain not changed." << std::endl;
    res->success = false;
  }

  if (req->a_d >= 0) {
    controllers_.at(called_dof).a_Kd = req->a_d;
  } else {
    msg << "Given actuator derivative gain is negative. Please give a positive number." <<
      std::endl << "Actuator derivative gain not changed." << std::endl;
    res->success = false;
  }

  if (req->j_p >= 0) {
    controllers_.at(called_dof).j_Kp = req->j_p;
  } else {
    msg << "Given joint proportional gain is negative. Please give a positive number." <<
      std::endl << "Joint proportional gain not changed." << std::endl;
    res->success = false;
  }

  if (req->j_d >= 0) {
    controllers_.at(called_dof).j_Kd = req->j_d;
  } else {
    msg << "Given joint derivative gain is negative. Please give a positive number." <<
      std::endl << "Joint derivative gain not changed." << std::endl;
    res->success = false;
  }

  if (req->i >= 0) {
    controllers_.at(called_dof).Ki = req->i;
  } else {
    msg << "Given integral gain is negative. Please give a positive number." << std::endl <<
      "Integral gain not changed." << std::endl;
    res->success = false;
  }

  msg << "Changed controller gains for" << std::endl << "joint number: " << called_dof <<
    std::endl << "joint name: " << links_.at(called_dof).name << "." << std::endl <<
    "Current gains are: " << std::endl << "a_Kp: " << controllers_.at(called_dof).a_Kp <<
    std::endl << "a_Kd: " << controllers_.at(called_dof).a_Kd << std::endl << "j_Kp: " <<
    controllers_.at(called_dof).j_Kp << std::endl << "j_Kd: " <<
    controllers_.at(called_dof).j_Kd << std::endl << "Ki: " << controllers_.at(called_dof).Ki <<
    std::endl;
  res->message = msg.str();
}

void ActuatorPositionController::updatePrint(
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

void ActuatorPositionController::getJointNames(
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

void ActuatorPositionController::setFeedForwardTorque(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
  std::shared_ptr<std_srvs::srv::SetBool::Response> res)
{
  // NOTE(ros2-port): ff_torque_enabled_ is written here (controller_manager
  // service-callback thread) and read every cycle in update() (RT control-loop
  // thread) with no lock -- unsynchronized, exactly as in the ROS1 controller.
  // See code review 2026-08-04.
  ff_torque_enabled_ = req->data;
  res->success = true;
  res->message = "Feedforward torque term was successfully " +
    std::string(ff_torque_enabled_ ? "enabled" : "disabled") + "!";
}

}  // namespace custom_controller

PLUGINLIB_EXPORT_CLASS(
  custom_controller::ActuatorPositionController, controller_interface::ControllerInterface)
