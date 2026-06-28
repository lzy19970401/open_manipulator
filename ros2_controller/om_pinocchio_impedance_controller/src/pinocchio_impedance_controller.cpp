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

#include <pinocchio_impedance_controller/pinocchio_impedance_controller.hpp>

#include <cmath>
#include <limits>
#include <stdexcept>

#include <controller_interface/helpers.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio_impedance_controller/impedance_control_law.hpp>
#include <rclcpp/rclcpp.hpp>

namespace pinocchio_impedance_controller
{

PinocchioImpedanceController::PinocchioImpedanceController()
: controller_interface::ControllerInterface()
{
}

controller_interface::InterfaceConfiguration
PinocchioImpedanceController::command_interface_configuration() const
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
PinocchioImpedanceController::state_interface_configuration() const
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

void PinocchioImpedanceController::fill_configuration_vector(Eigen::VectorXd & q) const
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

  if (model_.existJointName("gripper_left_joint") &&
    model_.existJointName("gripper_right_joint"))
  {
    const pinocchio::JointIndex left_id = model_.getJointId("gripper_left_joint");
    const pinocchio::JointIndex right_id = model_.getJointId("gripper_right_joint");
    q[model_.joints[right_id].idx_q()] = q[model_.joints[left_id].idx_q()];
  }
}

void PinocchioImpedanceController::fill_velocity_vector(Eigen::VectorXd & v) const
{
  v.setZero(model_.nv);

  for (size_t i = 0; i < n_joints_; ++i) {
    const double velocity =
      arm_state_interface_[1][i].get().get_optional().value_or(0.0);
    v[arm_velocity_indices_[i]] = velocity;
  }
}

void PinocchioImpedanceController::initialize_reference_target_from_mode()
{
  q_d_target_.resize(n_joints_);
  q_d_active_.resize(n_joints_);
  q_d_dot_active_.assign(n_joints_, 0.0);
  q_d_dot_prev_.assign(n_joints_, 0.0);
  q_d_ddot_active_.assign(n_joints_, 0.0);

  if (params_.reference_pose_mode == "fixed") {
    if (params_.fixed_reference_positions.size() != n_joints_) {
      throw std::runtime_error(
              "fixed_reference_positions must have length " + std::to_string(n_joints_));
    }
    q_d_target_ = params_.fixed_reference_positions;
  } else {
    for (size_t i = 0; i < n_joints_; ++i) {
      q_d_target_[i] =
        arm_state_interface_[0][i].get().get_optional().value_or(0.0);
    }
  }

  q_d_active_ = q_d_target_;
}

controller_interface::return_type PinocchioImpedanceController::update(
  [[maybe_unused]] const rclcpp::Time & time,
  const rclcpp::Duration & period)
{
  param_listener_->refresh_dynamic_parameters();
  params_ = param_listener_->get_params();

  const double dt = period.seconds();

  step_reference_interpolation(
    q_d_active_, q_d_dot_active_, q_d_target_, params_.reference_transition_max_velocity, dt);
  step_reference_acceleration(q_d_ddot_active_, q_d_dot_active_, q_d_dot_prev_, dt);
  q_d_dot_prev_ = q_d_dot_active_;

  Eigen::VectorXd q(model_.nq);
  Eigen::VectorXd v(model_.nv);
  fill_configuration_vector(q);
  fill_velocity_vector(v);

  Eigen::VectorXd ddq = Eigen::VectorXd::Zero(model_.nv);
  for (size_t i = 0; i < n_joints_; ++i) {
    ddq[arm_velocity_indices_[i]] = q_d_ddot_active_[i];
  }

  pinocchio::rnea(model_, *data_, q, v, ddq);

  const double friction_deadzone = params_.friction_velocity_deadzone;

  for (size_t i = 0; i < n_joints_; ++i) {
    const int velocity_index = arm_velocity_indices_[i];
    const int configuration_index = arm_configuration_indices_[i];
    const double q_meas = q[configuration_index];
    const double qdot_meas = v[velocity_index];

    const double friction_torque = compute_friction_torque(
      qdot_meas,
      params_.viscous_friction_coefficients[i],
      params_.coulomb_friction_coefficients[i],
      friction_deadzone,
      params_.enable_friction_compensation);

    const double effort = compute_joint_impedance_torque(
      data_->tau[velocity_index],
      q_meas,
      qdot_meas,
      q_d_active_[i],
      q_d_dot_active_[i],
      params_.stiffness[i],
      params_.damping[i],
      friction_torque,
      params_.effort_sign_flips[i],
      params_.torque_scaling_factors[i]);

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

controller_interface::CallbackReturn PinocchioImpedanceController::on_init()
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

controller_interface::CallbackReturn PinocchioImpedanceController::on_configure(
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

  const auto expect_length = [logger, n = n_joints_](const char * name, size_t len) -> bool {
      if (len != n) {
        RCLCPP_ERROR(logger, "'%s' must have length %zu", name, n);
        return false;
      }
      return true;
    };

  if (!expect_length("effort_sign_flips", params_.effort_sign_flips.size()) ||
    !expect_length("torque_scaling_factors", params_.torque_scaling_factors.size()) ||
    !expect_length("stiffness", params_.stiffness.size()) ||
    !expect_length("damping", params_.damping.size()) ||
    !expect_length(
      "reference_transition_max_velocity",
      params_.reference_transition_max_velocity.size()))
  {
    return CallbackReturn::ERROR;
  }

  if (params_.reference_pose_mode == "fixed" &&
    params_.fixed_reference_positions.size() != n_joints_)
  {
    RCLCPP_ERROR(
      logger, "fixed_reference_positions must have length %zu when reference_pose_mode is fixed",
      n_joints_);
    return CallbackReturn::ERROR;
  }

  if (params_.enable_friction_compensation &&
    (!expect_length(
      "viscous_friction_coefficients", params_.viscous_friction_coefficients.size()) ||
    !expect_length("coulomb_friction_coefficients", params_.coulomb_friction_coefficients.size())))
  {
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

  arm_velocity_indices_.clear();
  arm_configuration_indices_.clear();
  for (const auto & joint_name : joint_names_) {
    if (!model_.existJointName(joint_name)) {
      RCLCPP_ERROR(logger, "Arm joint '%s' not found in Pinocchio model", joint_name.c_str());
      return CallbackReturn::ERROR;
    }
    const pinocchio::JointIndex joint_id = model_.getJointId(joint_name);
    arm_velocity_indices_.push_back(model_.joints[joint_id].idx_v());
    arm_configuration_indices_.push_back(model_.joints[joint_id].idx_q());
  }

  passive_configuration_indices_.clear();
  for (const auto & joint_name : passive_joint_names_) {
    if (!model_.existJointName(joint_name)) {
      RCLCPP_ERROR(
        logger, "Passive state joint '%s' not found in Pinocchio model", joint_name.c_str());
      return CallbackReturn::ERROR;
    }
    const pinocchio::JointIndex joint_id = model_.getJointId(joint_name);
    passive_configuration_indices_.push_back(model_.joints[joint_id].idx_q());
  }

  joint_command_interface_.assign(command_interface_types_.size(), {});
  arm_state_interface_.assign(state_interface_types_.size(), {});
  passive_state_interface_.assign(1, {});

  RCLCPP_INFO(
    logger,
    "PinocchioImpedanceController configured (nq=%d, nv=%d, arm joints=%zu, passive=%zu, "
    "reference_pose_mode=%s)",
    model_.nq, model_.nv, n_joints_, passive_joint_names_.size(),
    params_.reference_pose_mode.c_str());

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PinocchioImpedanceController::on_activate(
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

  try {
    initialize_reference_target_from_mode();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger, "Failed to initialize reference pose: %s", e.what());
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    logger, "PinocchioImpedanceController activated (reference_pose_mode=%s)",
    params_.reference_pose_mode.c_str());
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PinocchioImpedanceController::on_deactivate(
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

  RCLCPP_INFO(get_node()->get_logger(), "PinocchioImpedanceController deactivated");
  return CallbackReturn::SUCCESS;
}

}  // namespace pinocchio_impedance_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  pinocchio_impedance_controller::PinocchioImpedanceController,
  controller_interface::ControllerInterface)
