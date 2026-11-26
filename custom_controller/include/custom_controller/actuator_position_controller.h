/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  Copyright (c) 2012, hiDOF, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*
 Author: Malgorzata Kamedula
 Desc: Effort position controller for the SEA with SEA simulation

 ROS2 port note (2026-08-04): this file replaces the four near-duplicate ROS1
 controllers actuator_position_controller1.cpp, _4.cpp, _all.cpp and
 _classes.cpp (all implemented the identical per-joint SEA position control
 law, just at different fixed/variable joint counts). This single
 ros2_control controller_interface::ControllerInterface plugin is
 parameterized by "dof" like the ROS1 "_classes"/"_all" variants were, and
 keeps that variant's control law and service behavior (including the
 FIXME-tagged findings from the ROS1 code review) unchanged.
*/

#ifndef CUSTOM_CONTROLLER__ACTUATOR_POSITION_CONTROLLER_H_
#define CUSTOM_CONTROLLER__ACTUATOR_POSITION_CONTROLLER_H_

#include <memory>
#include <sstream>
#include <string>
#include <atomic>
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

enum class JointType
{
  REVOLUTE,
  CONTINUOUS,
  PRISMATIC,
  UNKNOWN
};

class ActuatorPositionController : public controller_interface::ControllerInterface
{
public:
  ActuatorPositionController();

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

  // Required so that "dof"/"jointN.*"/"actuators.*" (arbitrary, dof-sized
  // parameter trees loaded from pandora_controllers.yaml) are usable via
  // get_parameter() without individually pre-declaring every dotted name --
  // matches how the ROS1 controller could read any XmlRpc parameter tree from
  // the parameter server without a declare step.
  rclcpp::NodeOptions define_custom_node_options() const override;

  struct Commands
  {
    std::vector<double> position;      // Last commanded position
    std::vector<double> velocity;      // Last commanded velocity
    std::vector<double> effort;        // Last commanded effort
    std::vector<double> online_gain1;  // Last commanded online gain 1
    std::vector<double> online_gain2;  // Last commanded online gain 2
  };

private:
  // Physical actuator (motor-side) model: identical integration scheme to the
  // ROS1 controller (semi-implicit Euler on a torsional-spring/SEA model).
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

  // Per-joint PID-like control law. a_Kp/a_Kd act on the motor side (as in
  // the ROS1 "_classes" variant). Fix (2026-10-12):
  //  - Ki integrates error*dt (Nm/(rad s)), not a per-sample sum, so it no
  //    longer depends on update_rate; clamped so |Ki*I| <= torque limit
  //    (anti-windup) and reset on activate.
  //  - with integral_on_link (default), the integrated error is the LINK
  //    position error -- removes the spring deflection under load (load/Ks),
  //    which the motor-side integral could never correct.
  //  - j_Kd: active damping on link velocity (was read but unused).
  //  j_Kp is still unused (non-collocated link stiffness; kept for config
  //  compatibility).
  class Controller
  {
  public:
    Controller(double a_Kp, double a_Kd, double j_Kp, double j_Kd, double Ki)
    : a_Kp(a_Kp), a_Kd(a_Kd), j_Kp(j_Kp), j_Kd(j_Kd), Ki(Ki),
      integrator_error(0), print(false), lower_limit(0), upper_limit(0),
      type(JointType::UNKNOWN)
    {
    }

    double a_Kp;
    double a_Kd;
    double j_Kp;
    double j_Kd;
    double Ki;
    double integrator_error;
    bool print;
    double lower_limit;
    double upper_limit;
    JointType type;

    void enforceStateLimits(double & command);
    double update(
      double desired_position, double feed_forward, double motor_position,
      double motor_velocity, double link_position, double link_velocity,
      double dt, double integral_limit, bool integral_on_link);
  };

  // Link (URDF joint) side: holds the loaned effort command interface and the
  // loaned position/velocity state interfaces for one joint, replacing the
  // ROS1 hardware_interface::JointHandle.
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

  // -- parameter loading / setup, mirrors the ROS1 init() sequence --
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

  // number of active (non-omitted) joints
  int dof_ = 0;
  // raw "dof" parameter, i.e. joint count before "omit" filtering -- kept as
  // the size of reversed_selector_, exactly like the ROS1 controller.
  int raw_dof_ = 0;

  int loop_count_ = 0;
  int update_rate_ = 1;
  // ~/state and ~/debug are published once every state_publish_every_ loops
  // (2026-11-25). The control law still runs every update_rate_ loops; at
  // 2 kHz, publishing every loop flooded the Python consumers.
  int state_publish_every_ = 1;
  bool ff_torque_enabled_ = false;
  bool integral_on_link_ = true;
  // Soft start (2026-11-24): on activation the commanded position starts at
  // each measured link position and slews to initial_position at
  // startup_rate_ (rad/s) until it gets there or the first ~/command message
  // arrives. 0 = old behaviour (step to initial_position on activation).
  double startup_rate_ = 0.0;
  bool startup_active_ = false;
  std::atomic<bool> command_received_{false};
  std::vector<double> startup_position_;

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

#endif  // CUSTOM_CONTROLLER__ACTUATOR_POSITION_CONTROLLER_H_
