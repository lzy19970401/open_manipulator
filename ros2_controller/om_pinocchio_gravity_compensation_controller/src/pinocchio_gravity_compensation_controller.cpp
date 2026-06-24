// Copyright 2026 OpenMANIPULATOR contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <pinocchio_gravity_compensation_controller/pinocchio_gravity_compensation_controller.hpp>

#include <cmath>
#include <limits>
#include <stdexcept>

#include <controller_interface/helpers.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <rclcpp/rclcpp.hpp>

namespace pinocchio_gravity_compensation_controller
{

PinocchioGravityCompensationController::PinocchioGravityCompensationController()
: controller_interface::ControllerInterface()
{
}

double PinocchioGravityCompensationController::sign_with_deadzone(
  double velocity, double deadzone)
{
  if (std::abs(velocity) < deadzone) {
    return 0.0;
  }
  return velocity > 0.0 ? 1.0 : -1.0;
}

controller_interface::InterfaceConfiguration
PinocchioGravityCompensationController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & joint_name : joint_names_) {
    for (const auto & interface_type : command_interface_types_) {
      config.names.push_back(joint_name + "/" + interface_type);
    }
  }

  return config;
}

controller_interface::InterfaceConfiguration
PinocchioGravityCompensationController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & joint_name : joint_names_) {
    for (const auto & interface_type : state_interface_types_) {
      config.names.push_back(joint_name + "/" + interface_type);
    }
  }

  for (const auto & joint_name : passive_joint_names_) {
    config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
  }

  return config;
}

void PinocchioGravityCompensationController::fill_configuration_vector(Eigen::VectorXd & q) const
{
  q = pinocchio::neutral(model_);

  for (size_t i = 0; i < n_joints_; ++i) {
    const double position =
      arm_state_interface_[0][i].get().get_optional().value_or(0.0);
    q[arm_configuration_indices_[i]] = position;
  }

  for (size_t i = 0; i < passive_joint_names_.size(); ++i) {
    const double position =
      passive_state_interface_[0][i].get().get_optional().value_or(0.0);
    q[passive_configuration_indices_[i]] = position;
  }

  // Hardware ros2_control exposes Master finger only; mirror Mimic finger for full q.
  if (model_.existJointName("gripper_left_joint") &&
    model_.existJointName("gripper_right_joint"))
  {
    const pinocchio::JointIndex left_id = model_.getJointId("gripper_left_joint");
    const pinocchio::JointIndex right_id = model_.getJointId("gripper_right_joint");
    q[model_.joints[right_id].idx_q()] = q[model_.joints[left_id].idx_q()];
  }
}

controller_interface::return_type PinocchioGravityCompensationController::update(
  [[maybe_unused]] const rclcpp::Time & time,
  [[maybe_unused]] const rclcpp::Duration & period)
{
  Eigen::VectorXd q(model_.nq);
  fill_configuration_vector(q);

  pinocchio::computeGeneralizedGravity(model_, *data_, q);

  const double friction_deadzone = params_.friction_velocity_deadzone;

  for (size_t i = 0; i < n_joints_; ++i) {
    double tau_phys = data_->g[arm_velocity_indices_[i]];

    if (params_.enable_friction_compensation) {
      const double qdot =
        arm_state_interface_[1][i].get().get_optional().value_or(0.0);
      const double fv = params_.viscous_friction_coefficients[i];
      const double fc = params_.coulomb_friction_coefficients[i];
      tau_phys += fv * qdot + fc * sign_with_deadzone(qdot, friction_deadzone);
    }

    const double effort =
      params_.effort_sign_flips[i] * params_.torque_scaling_factors[i] * tau_phys;

    if (!std::isfinite(effort)) {
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 2000,
        "Non-finite effort on joint %zu; commanding 0.0", i);
    }

    const bool set_ok = joint_command_interface_[0][i].get().set_value(
      std::isfinite(effort) ? effort : 0.0);
    if (!set_ok) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Failed to set command value for joint %zu", i);
    }
  }

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn PinocchioGravityCompensationController::on_init()
{
  try {
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during init: %s", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PinocchioGravityCompensationController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto logger = get_node()->get_logger();

  if (!param_listener_) {
    RCLCPP_ERROR(logger, "Parameter listener missing after init");
    return CallbackReturn::ERROR;
  }

  param_listener_->refresh_dynamic_parameters();
  params_ = param_listener_->get_params();

  joint_names_ = params_.joints;
  passive_joint_names_ = params_.passive_state_joints;
  n_joints_ = joint_names_.size();

  if (n_joints_ == 0) {
    RCLCPP_ERROR(logger, "'joints' parameter is empty");
    return CallbackReturn::ERROR;
  }

  if (params_.effort_sign_flips.size() != n_joints_ ||
    params_.torque_scaling_factors.size() != n_joints_)
  {
    RCLCPP_ERROR(
      logger,
      "effort_sign_flips and torque_scaling_factors must have length %zu", n_joints_);
    return CallbackReturn::ERROR;
  }

  if (params_.enable_friction_compensation &&
    (params_.viscous_friction_coefficients.size() != n_joints_ ||
    params_.coulomb_friction_coefficients.size() != n_joints_))
  {
    RCLCPP_ERROR(
      logger,
      "Friction coefficient arrays must have length %zu when friction compensation is enabled",
      n_joints_);
    return CallbackReturn::ERROR;
  }

  const std::string & urdf = get_robot_description();
  if (urdf.empty()) {
    RCLCPP_ERROR(logger, "robot_description is empty; cannot build Pinocchio model");
    return CallbackReturn::ERROR;
  }

  try {
    pinocchio::urdf::buildModelFromXML(urdf, model_);
    data_ = std::make_unique<pinocchio::Data>(model_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger, "Failed to build Pinocchio model from robot_description: %s", e.what());
    return CallbackReturn::ERROR;
  }

  arm_joint_ids_.clear();
  arm_velocity_indices_.clear();
  arm_configuration_indices_.clear();
  for (const auto & joint_name : joint_names_) {
    if (!model_.existJointName(joint_name)) {
      RCLCPP_ERROR(logger, "Arm joint '%s' not found in Pinocchio model", joint_name.c_str());
      return CallbackReturn::ERROR;
    }
    const pinocchio::JointIndex joint_id = model_.getJointId(joint_name);
    arm_joint_ids_.push_back(joint_id);
    arm_velocity_indices_.push_back(model_.joints[joint_id].idx_v());
    arm_configuration_indices_.push_back(model_.joints[joint_id].idx_q());
  }

  passive_joint_ids_.clear();
  passive_configuration_indices_.clear();
  for (const auto & joint_name : passive_joint_names_) {
    if (!model_.existJointName(joint_name)) {
      RCLCPP_ERROR(
        logger, "Passive state joint '%s' not found in Pinocchio model", joint_name.c_str());
      return CallbackReturn::ERROR;
    }
    const pinocchio::JointIndex joint_id = model_.getJointId(joint_name);
    passive_joint_ids_.push_back(joint_id);
    passive_configuration_indices_.push_back(model_.joints[joint_id].idx_q());
  }

  joint_command_interface_.assign(command_interface_types_.size(), {});
  arm_state_interface_.assign(state_interface_types_.size(), {});
  passive_state_interface_.assign(1, {});

  RCLCPP_INFO(
    logger,
    "PinocchioGravityCompensationController configured (nq=%d, nv=%d, arm joints=%zu, passive=%zu)",
    model_.nq, model_.nv, n_joints_, passive_joint_names_.size());

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PinocchioGravityCompensationController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto logger = get_node()->get_logger();

  param_listener_->refresh_dynamic_parameters();
  params_ = param_listener_->get_params();

  for (const auto & interface : params_.command_interfaces) {
    const auto it =
      std::find(command_interface_types_.begin(), command_interface_types_.end(), interface);
    const auto index = static_cast<size_t>(std::distance(command_interface_types_.begin(), it));
    if (!controller_interface::get_ordered_interfaces(
        command_interfaces_, joint_names_, interface, joint_command_interface_[index]))
    {
      RCLCPP_ERROR(
        logger, "Expected %zu '%s' command interfaces, got %zu.", n_joints_, interface.c_str(),
        joint_command_interface_[index].size());
      return CallbackReturn::ERROR;
    }
  }

  for (const auto & interface : params_.state_interfaces) {
    const auto it =
      std::find(state_interface_types_.begin(), state_interface_types_.end(), interface);
    const auto index = static_cast<size_t>(std::distance(state_interface_types_.begin(), it));
    if (!controller_interface::get_ordered_interfaces(
        state_interfaces_, joint_names_, interface, arm_state_interface_[index]))
    {
      RCLCPP_ERROR(
        logger, "Expected %zu '%s' state interfaces, got %zu.", n_joints_, interface.c_str(),
        arm_state_interface_[index].size());
      return CallbackReturn::ERROR;
    }
  }

  if (!passive_joint_names_.empty()) {
    if (!controller_interface::get_ordered_interfaces(
        state_interfaces_, passive_joint_names_, hardware_interface::HW_IF_POSITION,
        passive_state_interface_[0]))
    {
      RCLCPP_ERROR(
        logger, "Expected %zu passive position state interfaces, got %zu.",
        passive_joint_names_.size(), passive_state_interface_[0].size());
      return CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(logger, "PinocchioGravityCompensationController activated");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PinocchioGravityCompensationController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (size_t i = 0; i < n_joints_; ++i) {
    for (size_t j = 0; j < command_interface_types_.size(); ++j) {
      const bool set_ok = command_interfaces_[i * command_interface_types_.size() + j].set_value(0.0);
      if (!set_ok) {
        RCLCPP_ERROR(
          get_node()->get_logger(),
          "Failed to reset command value for joint %zu, interface %zu", i, j);
      }
    }
  }

  RCLCPP_INFO(get_node()->get_logger(), "PinocchioGravityCompensationController deactivated");
  return CallbackReturn::SUCCESS;
}

}  // namespace pinocchio_gravity_compensation_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  pinocchio_gravity_compensation_controller::PinocchioGravityCompensationController,
  controller_interface::ControllerInterface)
