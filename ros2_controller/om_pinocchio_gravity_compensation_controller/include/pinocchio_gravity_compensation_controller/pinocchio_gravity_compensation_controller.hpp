#ifndef PINOCCHIO_GRAVITY_COMPENSATION_CONTROLLER__PINOCCHIO_GRAVITY_COMPENSATION_CONTROLLER_HPP_
#define PINOCCHIO_GRAVITY_COMPENSATION_CONTROLLER__PINOCCHIO_GRAVITY_COMPENSATION_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "pinocchio_gravity_compensation_controller/visibility_control.h"

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "om_pinocchio_gravity_compensation_controller/pinocchio_gravity_compensation_controller_parameters.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/multibody/model.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace pinocchio_gravity_compensation_controller
{

class PinocchioGravityCompensationController : public controller_interface::ControllerInterface
{
public:
  PINOCCHIO_GRAVITY_COMPENSATION_CONTROLLER_PUBLIC
  PinocchioGravityCompensationController();

  PINOCCHIO_GRAVITY_COMPENSATION_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  PINOCCHIO_GRAVITY_COMPENSATION_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  PINOCCHIO_GRAVITY_COMPENSATION_CONTROLLER_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  PINOCCHIO_GRAVITY_COMPENSATION_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  PINOCCHIO_GRAVITY_COMPENSATION_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  PINOCCHIO_GRAVITY_COMPENSATION_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  PINOCCHIO_GRAVITY_COMPENSATION_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

private:
  static double sign_with_deadzone(double velocity, double deadzone);

  void fill_configuration_vector(Eigen::VectorXd & q) const;

  std::shared_ptr<ParamListener> param_listener_;
  Params params_;

  pinocchio::Model model_;
  std::unique_ptr<pinocchio::Data> data_;

  std::vector<std::string> joint_names_;
  std::vector<std::string> passive_joint_names_;
  size_t n_joints_{0};

  std::vector<pinocchio::JointIndex> arm_joint_ids_;
  std::vector<pinocchio::JointIndex> passive_joint_ids_;
  std::vector<int> arm_velocity_indices_;
  std::vector<int> arm_configuration_indices_;
  std::vector<int> passive_configuration_indices_;

  std::vector<std::string> state_interface_types_ = {
    hardware_interface::HW_IF_POSITION,
    hardware_interface::HW_IF_VELOCITY,
  };
  std::vector<std::string> command_interface_types_ = {
    hardware_interface::HW_IF_EFFORT
  };

  template<typename T>
  using InterfaceReferences = std::vector<std::vector<std::reference_wrapper<T>>>;

  InterfaceReferences<hardware_interface::LoanedCommandInterface> joint_command_interface_;
  InterfaceReferences<hardware_interface::LoanedStateInterface> arm_state_interface_;
  InterfaceReferences<hardware_interface::LoanedStateInterface> passive_state_interface_;
};

}  // namespace pinocchio_gravity_compensation_controller

#endif  // PINOCCHIO_GRAVITY_COMPENSATION_CONTROLLER__PINOCCHIO_GRAVITY_COMPENSATION_CONTROLLER_HPP_
