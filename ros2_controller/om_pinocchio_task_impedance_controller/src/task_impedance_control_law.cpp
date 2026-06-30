// Copyright 2026 OpenMANIPULATOR contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include <pinocchio_task_impedance_controller/task_impedance_control_law.hpp>

#include <om_impedance_utilities/impedance_utilities.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

namespace pinocchio_task_impedance_controller
{

Eigen::Vector3d compute_ee_position_in_base_frame(
  const pinocchio::Model & model,
  pinocchio::Data & data,
  const Eigen::VectorXd & q,
  pinocchio::FrameIndex base_frame_id,
  pinocchio::FrameIndex ee_frame_id)
{
  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);
  return data.oMf[base_frame_id].actInv(data.oMf[ee_frame_id]).translation();
}

void compute_position_jacobian_in_base_frame(
  const pinocchio::Model & model,
  pinocchio::Data & data,
  const Eigen::VectorXd & q,
  pinocchio::FrameIndex base_frame_id,
  pinocchio::FrameIndex ee_frame_id,
  Eigen::Ref<Eigen::MatrixXd> J_p)
{
  pinocchio::forwardKinematics(model, data, q);
  pinocchio::computeJointJacobians(model, data, q);
  pinocchio::updateFramePlacements(model, data);

  Eigen::MatrixXd J6(6, model.nv);
  pinocchio::getFrameJacobian(
    model, data, ee_frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J6);

  const Eigen::Matrix3d R_base_world = data.oMf[base_frame_id].rotation();
  J_p = R_base_world.transpose() * J6.topRows<3>();
}

void compute_position_jacobian_time_variation_in_base_frame(
  const pinocchio::Model & model,
  pinocchio::Data & data,
  const Eigen::VectorXd & q,
  const Eigen::VectorXd & v,
  pinocchio::FrameIndex base_frame_id,
  pinocchio::FrameIndex ee_frame_id,
  Eigen::Ref<Eigen::MatrixXd> dJ_p)
{
  pinocchio::forwardKinematics(model, data, q, v);
  pinocchio::computeJointJacobians(model, data, q);
  pinocchio::computeJointJacobiansTimeVariation(model, data, q, v);
  pinocchio::updateFramePlacements(model, data);

  Eigen::MatrixXd dJ6(6, model.nv);
  pinocchio::getFrameJacobianTimeVariation(
    model, data, ee_frame_id, pinocchio::LOCAL_WORLD_ALIGNED, dJ6);

  const Eigen::Matrix3d R_base_world = data.oMf[base_frame_id].rotation();
  dJ_p = R_base_world.transpose() * dJ6.topRows<3>();
}

Eigen::VectorXd compute_null_space_desired_acceleration(
  const pinocchio::Model & model,
  const Eigen::VectorXd & q,
  const std::vector<double> & q_capture,
  const std::vector<double> & null_space_stiffness,
  const std::vector<int> & arm_configuration_indices,
  const std::vector<int> & arm_velocity_indices)
{
  Eigen::VectorXd qdd_null = Eigen::VectorXd::Zero(model.nv);
  for (size_t i = 0; i < arm_velocity_indices.size(); ++i) {
    const double delta_q =
      q_capture[i] - q[arm_configuration_indices[i]];
    qdd_null[arm_velocity_indices[i]] = null_space_stiffness[i] * delta_q;
  }
  return qdd_null;
}

void fill_joint_acceleration_bounds(
  const std::vector<double> & joint_acceleration_limits,
  const std::vector<int> & arm_velocity_indices,
  Eigen::Ref<Eigen::VectorXd> qdd_min,
  Eigen::Ref<Eigen::VectorXd> qdd_max)
{
  qdd_min.setZero();
  qdd_max.setZero();

  for (size_t i = 0; i < arm_velocity_indices.size(); ++i) {
    const int idx = arm_velocity_indices[i];
    qdd_min[idx] = -joint_acceleration_limits[i];
    qdd_max[idx] = joint_acceleration_limits[i];
  }
}

TaskImpedanceTorqueResult compute_task_impedance_torques_v1(
  const pinocchio::Model & model,
  pinocchio::Data & data,
  const Eigen::VectorXd & q,
  const Eigen::VectorXd & v,
  const Eigen::Vector3d & x_d_active,
  const Eigen::Vector3d & x_d_dot_active,
  const Eigen::Vector3d & K_p,
  const Eigen::Vector3d & D_p,
  pinocchio::FrameIndex base_frame_id,
  pinocchio::FrameIndex ee_frame_id,
  const std::vector<int> & arm_velocity_indices,
  const std::vector<double> & effort_sign_flips,
  const std::vector<double> & torque_scaling_factors)
{
  TaskImpedanceTorqueResult result;
  result.arm_torques.resize(arm_velocity_indices.size(), 0.0);
  result.desired_joint_acceleration = Eigen::VectorXd::Zero(model.nv);

  result.end_effector_position =
    compute_ee_position_in_base_frame(model, data, q, base_frame_id, ee_frame_id);

  Eigen::MatrixXd J_p(3, model.nv);
  compute_position_jacobian_in_base_frame(
    model, data, q, base_frame_id, ee_frame_id, J_p);
  result.end_effector_velocity = J_p * v;

  result.cartesian_force = compute_cartesian_spring_force(
    x_d_active, result.end_effector_position, x_d_dot_active, result.end_effector_velocity,
    K_p, D_p);

  const Eigen::VectorXd ddq = Eigen::VectorXd::Zero(model.nv);
  pinocchio::rnea(model, data, q, v, ddq);

  const Eigen::VectorXd tau_full = data.tau + J_p.transpose() * result.cartesian_force;

  for (size_t i = 0; i < arm_velocity_indices.size(); ++i) {
    result.arm_torques[i] = om_impedance_utilities::apply_effort_sign_and_scaling(
      tau_full[arm_velocity_indices[i]], effort_sign_flips[i], torque_scaling_factors[i]);
  }

  result.qp_success = true;
  return result;
}

TaskImpedanceTorqueResult compute_task_impedance_torques(
  const pinocchio::Model & model,
  pinocchio::Data & data,
  const Eigen::VectorXd & q,
  const Eigen::VectorXd & v,
  const Eigen::Vector3d & x_d_active,
  const Eigen::Vector3d & x_d_dot_active,
  const Eigen::Vector3d & x_d_ddot_active,
  const std::vector<double> & q_capture,
  const Eigen::Vector3d & K_p,
  const Eigen::Vector3d & D_p,
  const std::vector<double> & null_space_stiffness,
  const std::vector<double> & joint_acceleration_limits,
  double qp_task_weight,
  double qp_null_weight,
  bool enable_friction_compensation,
  double friction_velocity_deadzone,
  const std::vector<double> & viscous_friction_coefficients,
  const std::vector<double> & coulomb_friction_coefficients,
  pinocchio::FrameIndex base_frame_id,
  pinocchio::FrameIndex ee_frame_id,
  const std::vector<int> & arm_velocity_indices,
  const std::vector<int> & arm_configuration_indices,
  const std::vector<double> & effort_sign_flips,
  const std::vector<double> & torque_scaling_factors,
  TaskImpedanceQpSolver & qp_solver)
{
  TaskImpedanceTorqueResult result;
  result.arm_torques.resize(arm_velocity_indices.size(), 0.0);

  result.end_effector_position =
    compute_ee_position_in_base_frame(model, data, q, base_frame_id, ee_frame_id);

  Eigen::MatrixXd J_p(3, model.nv);
  compute_position_jacobian_in_base_frame(
    model, data, q, base_frame_id, ee_frame_id, J_p);
  result.end_effector_velocity = J_p * v;

  result.cartesian_force = compute_cartesian_spring_force(
    x_d_active, result.end_effector_position, x_d_dot_active, result.end_effector_velocity,
    K_p, D_p);

  Eigen::MatrixXd dJ_p(3, model.nv);
  compute_position_jacobian_time_variation_in_base_frame(
    model, data, q, v, base_frame_id, ee_frame_id, dJ_p);
  const Eigen::Vector3d a_des = x_d_ddot_active - dJ_p * v;

  const Eigen::VectorXd qdd_null = compute_null_space_desired_acceleration(
    model, q, q_capture, null_space_stiffness, arm_configuration_indices, arm_velocity_indices);

  Eigen::VectorXd qdd_min(model.nv);
  Eigen::VectorXd qdd_max(model.nv);
  fill_joint_acceleration_bounds(
    joint_acceleration_limits, arm_velocity_indices, qdd_min, qdd_max);

  TaskImpedanceQpInput qp_input;
  qp_input.J_p = J_p;
  qp_input.a_des = a_des;
  qp_input.qdd_null = qdd_null;
  qp_input.qp_task_weight = qp_task_weight;
  qp_input.qp_null_weight = qp_null_weight;
  qp_input.qdd_min = qdd_min;
  qp_input.qdd_max = qdd_max;

  const auto qp_result = qp_solver.solve(qp_input);
  result.qp_success = qp_result.success;
  result.desired_joint_acceleration =
    qp_result.success ? qp_result.qdd_d : Eigen::VectorXd::Zero(model.nv);

  pinocchio::rnea(model, data, q, v, result.desired_joint_acceleration);

  const Eigen::VectorXd tau_full =
    data.tau + J_p.transpose() * result.cartesian_force;

  for (size_t i = 0; i < arm_velocity_indices.size(); ++i) {
    const int velocity_index = arm_velocity_indices[i];
    const double friction_torque = om_impedance_utilities::compute_friction_torque(
      v[velocity_index],
      viscous_friction_coefficients[i],
      coulomb_friction_coefficients[i],
      friction_velocity_deadzone,
      enable_friction_compensation);

    const double tau_phys = tau_full[velocity_index] + friction_torque;
    result.arm_torques[i] = om_impedance_utilities::apply_effort_sign_and_scaling(
      tau_phys, effort_sign_flips[i], torque_scaling_factors[i]);
  }

  return result;
}

}  // namespace pinocchio_task_impedance_controller
