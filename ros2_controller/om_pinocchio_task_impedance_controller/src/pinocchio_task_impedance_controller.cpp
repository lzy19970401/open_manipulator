// Copyright 2026 OpenMANIPULATOR contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include <pinocchio_task_impedance_controller/pinocchio_task_impedance_controller.hpp>

#include <cmath>
#include <stdexcept>

#include <controller_interface/helpers.hpp>
#include <om_impedance_utilities/impedance_utilities.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio_task_impedance_controller/task_impedance_control_law.hpp>
#include <pinocchio_task_impedance_controller/task_impedance_qp_solver.hpp>
#include <rclcpp/rclcpp.hpp>

namespace pinocchio_task_impedance_controller
{

namespace
{

Eigen::Vector3d to_eigen3(const std::array<double, 3> & values)
{
  return Eigen::Vector3d(values[0], values[1], values[2]);
}

std::array<double, 3> to_array3(const Eigen::Vector3d & values)
{
  return {{values[0], values[1], values[2]}};
}

std::vector<double> to_vector3(const std::array<double, 3> & values)
{
  return {values[0], values[1], values[2]};
}

}  // namespace

PinocchioTaskImpedanceController::PinocchioTaskImpedanceController()
: controller_interface::ControllerInterface()
{
}

PinocchioTaskImpedanceController::~PinocchioTaskImpedanceController() = default;

controller_interface::InterfaceConfiguration
PinocchioTaskImpedanceController::command_interface_configuration() const
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
PinocchioTaskImpedanceController::state_interface_configuration() const
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

void PinocchioTaskImpedanceController::fill_configuration_vector(Eigen::VectorXd & q) const
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

void PinocchioTaskImpedanceController::fill_velocity_vector(Eigen::VectorXd & v) const
{
  v.setZero(model_.nv);

  for (size_t i = 0; i < n_joints_; ++i) {
    const double velocity =
      arm_state_interface_[1][i].get().get_optional().value_or(0.0);
    v[arm_velocity_indices_[i]] = velocity;
  }
}

Eigen::Vector3d PinocchioTaskImpedanceController::current_ee_position_in_base_frame() const
{
  Eigen::VectorXd q(model_.nq);
  fill_configuration_vector(q);
  return compute_ee_position_in_base_frame(model_, *data_, q, base_frame_id_, ee_frame_id_);
}

void PinocchioTaskImpedanceController::initialize_reference_target_from_mode()
{
  x_d_dot_active_.fill(0.0);
  x_d_dot_prev_.fill(0.0);
  x_d_ddot_active_.fill(0.0);

  if (params_.reference_position_mode == "fixed") {
    if (params_.fixed_reference_position.size() != 3) {
      throw std::runtime_error(
              "fixed_reference_position must have length 3 when reference_position_mode is fixed");
    }
    for (size_t i = 0; i < 3; ++i) {
      x_d_target_[i] = params_.fixed_reference_position[i];
    }
  } else {
    const Eigen::Vector3d x_capture = current_ee_position_in_base_frame();
    x_d_target_ = to_array3(x_capture);
  }

  x_d_active_ = x_d_target_;
}

controller_interface::return_type PinocchioTaskImpedanceController::update(
  [[maybe_unused]] const rclcpp::Time & time,
  const rclcpp::Duration & period)
{
  param_listener_->refresh_dynamic_parameters();
  params_ = param_listener_->get_params();

  const double dt = period.seconds();

  std::vector<double> x_d_active_vec = to_vector3(x_d_active_);
  std::vector<double> x_d_dot_active_vec = to_vector3(x_d_dot_active_);
  const std::vector<double> x_d_target_vec = to_vector3(x_d_target_);
  const std::vector<double> max_velocity(
    params_.reference_transition_max_velocity.begin(),
    params_.reference_transition_max_velocity.end());

  om_impedance_utilities::step_reference_interpolation(
    x_d_active_vec, x_d_dot_active_vec, x_d_target_vec, max_velocity, dt);

  std::vector<double> x_d_ddot_active_vec(3, 0.0);
  const std::vector<double> x_d_dot_prev_vec = to_vector3(x_d_dot_prev_);
  om_impedance_utilities::step_reference_acceleration(
    x_d_ddot_active_vec, x_d_dot_active_vec, x_d_dot_prev_vec, dt);

  for (size_t i = 0; i < 3; ++i) {
    x_d_active_[i] = x_d_active_vec[i];
    x_d_dot_active_[i] = x_d_dot_active_vec[i];
    x_d_ddot_active_[i] = x_d_ddot_active_vec[i];
  }
  x_d_dot_prev_ = x_d_dot_active_;

  Eigen::VectorXd q(model_.nq);
  Eigen::VectorXd v(model_.nv);
  fill_configuration_vector(q);
  fill_velocity_vector(v);

  if (params_.stiffness.size() != 3 || params_.damping.size() != 3) {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 2000,
      "stiffness and damping must each have length 3");
    return controller_interface::return_type::ERROR;
  }

  if (params_.null_space_stiffness.size() != n_joints_ ||
    params_.joint_acceleration_limits.size() != n_joints_)
  {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 2000,
      "null_space_stiffness and joint_acceleration_limits must each have length %zu", n_joints_);
    return controller_interface::return_type::ERROR;
  }

  const Eigen::Vector3d K_p(
    params_.stiffness[0], params_.stiffness[1], params_.stiffness[2]);
  const Eigen::Vector3d D_p(
    params_.damping[0], params_.damping[1], params_.damping[2]);

  if (!qp_solver_) {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 2000,
      "QP solver not initialized");
    return controller_interface::return_type::ERROR;
  }

  const auto result = compute_task_impedance_torques(
    model_, *data_, q, v, to_eigen3(x_d_active_), to_eigen3(x_d_dot_active_),
    to_eigen3(x_d_ddot_active_), q_capture_, K_p, D_p, params_.null_space_stiffness,
    params_.joint_acceleration_limits, params_.qp_task_weight, params_.qp_null_weight,
    params_.enable_friction_compensation, params_.friction_velocity_deadzone,
    params_.viscous_friction_coefficients, params_.coulomb_friction_coefficients,
    base_frame_id_, ee_frame_id_, arm_velocity_indices_, arm_configuration_indices_,
    params_.effort_sign_flips, params_.torque_scaling_factors, *qp_solver_);

  if (!result.qp_success) {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 2000,
      "ProxQP failed to solve for qdd_d; commanding zero effort");
    for (size_t i = 0; i < n_joints_; ++i) {
      (void)joint_command_interface_[0][i].get().set_value(0.0);
    }
    return controller_interface::return_type::OK;
  }

  for (size_t i = 0; i < n_joints_; ++i) {
    const double effort = result.arm_torques[i];

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

controller_interface::CallbackReturn PinocchioTaskImpedanceController::on_init()
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

controller_interface::CallbackReturn PinocchioTaskImpedanceController::on_configure(
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
    !expect_length("null_space_stiffness", params_.null_space_stiffness.size()) ||
    !expect_length("joint_acceleration_limits", params_.joint_acceleration_limits.size()))
  {
    return CallbackReturn::ERROR;
  }

  if (params_.enable_friction_compensation &&
    (!expect_length("viscous_friction_coefficients", params_.viscous_friction_coefficients.size()) ||
    !expect_length("coulomb_friction_coefficients", params_.coulomb_friction_coefficients.size())))
  {
    return CallbackReturn::ERROR;
  }

  if (params_.stiffness.size() != 3 || params_.damping.size() != 3) {
    RCLCPP_ERROR(logger, "'stiffness' and 'damping' must each have length 3");
    return CallbackReturn::ERROR;
  }

  if (params_.reference_transition_max_velocity.size() != 3) {
    RCLCPP_ERROR(logger, "'reference_transition_max_velocity' must have length 3");
    return CallbackReturn::ERROR;
  }

  if (params_.reference_position_mode == "fixed" &&
    params_.fixed_reference_position.size() != 3)
  {
    RCLCPP_ERROR(
      logger,
      "fixed_reference_position must have length 3 when reference_position_mode is fixed");
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
    qp_solver_ = std::make_unique<TaskImpedanceQpSolver>(model_.nv);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger, "Failed to build Pinocchio model from robot_description: %s", e.what());
    return CallbackReturn::ERROR;
  }

  if (!model_.existFrame(params_.base_frame)) {
    RCLCPP_ERROR(logger, "Base frame '%s' not found in Pinocchio model", params_.base_frame.c_str());
    return CallbackReturn::ERROR;
  }
  if (!model_.existFrame(params_.end_effector_frame)) {
    RCLCPP_ERROR(
      logger, "End effector frame '%s' not found in Pinocchio model",
      params_.end_effector_frame.c_str());
    return CallbackReturn::ERROR;
  }

  base_frame_id_ = model_.getFrameId(params_.base_frame);
  ee_frame_id_ = model_.getFrameId(params_.end_effector_frame);

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
    "PinocchioTaskImpedanceController configured (nq=%d, nv=%d, arm joints=%zu, passive=%zu, "
    "reference_position_mode=%s, base=%s, ee=%s)",
    model_.nq, model_.nv, n_joints_, passive_joint_names_.size(),
    params_.reference_position_mode.c_str(), params_.base_frame.c_str(),
    params_.end_effector_frame.c_str());

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PinocchioTaskImpedanceController::on_activate(
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

  q_capture_.resize(n_joints_);
  for (size_t i = 0; i < n_joints_; ++i) {
    q_capture_[i] =
      arm_state_interface_[0][i].get().get_optional().value_or(0.0);
  }

  RCLCPP_INFO(
    logger,
    "Posture capture q_capture = [%.4f, %.4f, %.4f, %.4f]",
    q_capture_[0], q_capture_[1], q_capture_[2], q_capture_[3]);

  try {
    initialize_reference_target_from_mode();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger, "Failed to initialize reference position: %s", e.what());
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    logger,
    "PinocchioTaskImpedanceController activated (reference_position_mode=%s, "
    "x_d_target=[%.4f, %.4f, %.4f], x_d_active=[%.4f, %.4f, %.4f])",
    params_.reference_position_mode.c_str(), x_d_target_[0], x_d_target_[1], x_d_target_[2],
    x_d_active_[0], x_d_active_[1], x_d_active_[2]);

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PinocchioTaskImpedanceController::on_deactivate(
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

  RCLCPP_INFO(get_node()->get_logger(), "PinocchioTaskImpedanceController deactivated");
  return CallbackReturn::SUCCESS;
}

}  // namespace pinocchio_task_impedance_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  pinocchio_task_impedance_controller::PinocchioTaskImpedanceController,
  controller_interface::ControllerInterface)
