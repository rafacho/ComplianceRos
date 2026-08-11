#ifndef CUSTOM_CONTROLLER__CONTROLLER_UTILS_H_
#define CUSTOM_CONTROLLER__CONTROLLER_UTILS_H_

// ROS2 port note (2026-08-04): MapToUrdf was never referenced by any of the
// actual controller plugins in ROS1 either (confirmed by grep across the
// package) -- it is ported as a standalone utility with the same behavior,
// not wired into anything, exactly as before.

#include <climits>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "custom_services/srv/get_joint_names.hpp"

namespace custom_controller
{

const int NON_EXISTING = INT_MAX;

/** @brief this class provides mappings between dofs as defined in a controller
 * yaml file for loaded joints and loaded urdf file **/
class MapToUrdf
{
private:
  bool valid_ = false;
  std::vector<int> id_;
  int dofs_ = 0;
  std::vector<std::string> controller_names_;
  std::vector<std::string> urdf_names_;
  void initCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

public:
  explicit MapToUrdf(
    const rclcpp::Node::SharedPtr & node, std::string topic = "joint_states",
    std::string controller = "position_controller");
  ~MapToUrdf() {}
  std::vector<int> getMapping() {return id_;}

  /**
   * @brief the function provides mapping between controller and
   * urdf (joint_states feedback) values, as the urdf vector may be bigger
   * than the controller vector (floating base case), it doesn't overwrite
   * values not present in urdf
   *
   * @note q_mapped must be of the proper size, it is not checked for
   * performance purposes, if q_mapped is too small it may cause memory
   * violation
   */
  template<typename Vector>
  void mapToController(Vector q_urdf, Vector & q_mapped)
  {
    for (int i = 0; i < dofs_; i++) {
      if (id_[i] != NON_EXISTING && id_[i] < static_cast<int>(q_urdf.size())) {
        q_mapped[i] = q_urdf[id_[i]];
      }
    }
  }

  /**
   * @brief the function provides mapping between controller and
   * urdf (joint_states feedback) values, as the urdf vector may be bigger
   * than the controller vector (floating base case), it doesn't overwrite
   * values not present in controller
   *
   * @note q_urdf must be of the proper size, it is not checked for
   * performance purposes, if q_urdf is too small it may cause memory
   * violation
   */
  template<typename Vector>
  void mapFromController(Vector q_controller, Vector & q_urdf)
  {
    for (int i = 0; i < dofs_; i++) {
      if (id_[i] != NON_EXISTING && id_[i] < static_cast<int>(q_urdf.size())) {
        q_urdf[id_[i]] = q_controller[i];
      }
    }
  }

  std::vector<std::string> getControllerNames() {return controller_names_;}
  std::vector<std::string> getUrdfNames() {return urdf_names_;}
  int getControllerDofs() {return dofs_;}
};

}  // namespace custom_controller

#endif  // CUSTOM_CONTROLLER__CONTROLLER_UTILS_H_
