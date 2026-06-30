#ifndef PINOCCHIO_TASK_IMPEDANCE_CONTROLLER__TASK_IMPEDANCE_CONTROL_LAW_HPP_
#define PINOCCHIO_TASK_IMPEDANCE_CONTROLLER__TASK_IMPEDANCE_CONTROL_LAW_HPP_

#include <cstddef>
#include <vector>

#include <Eigen/Core>
#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/fwd.hpp>
#include <pinocchio_task_impedance_controller/task_impedance_qp_solver.hpp>

namespace pinocchio_task_impedance_controller
{

inline Eigen::Vector3d compute_cartesian_spring_force(
  const Eigen::Vector3d & x_d,
  const Eigen::Vector3d & x,
  const Eigen::Vector3d & x_d_dot,
  const Eigen::Vector3d & x_dot,
  const Eigen::Vector3d & K_p,
  const Eigen::Vector3d & D_p)
{
  return K_p.cwiseProduct(x_d - x) + D_p.cwiseProduct(x_d_dot - x_dot);
}

struct TaskImpedanceTorqueResult
{
  std::vector<double> arm_torques;
  Eigen::Vector3d cartesian_force{Eigen::Vector3d::Zero()};
  Eigen::Vector3d end_effector_position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d end_effector_velocity{Eigen::Vector3d::Zero()};
  Eigen::VectorXd desired_joint_acceleration;
  bool qp_success{false};
};

Eigen::Vector3d compute_ee_position_in_base_frame(
  const pinocchio::Model & model,
  pinocchio::Data & data,
  const Eigen::VectorXd & q,
  pinocchio::FrameIndex base_frame_id,
  pinocchio::FrameIndex ee_frame_id);

void compute_position_jacobian_in_base_frame(
  const pinocchio::Model & model,
  pinocchio::Data & data,
  const Eigen::VectorXd & q,
  pinocchio::FrameIndex base_frame_id,
  pinocchio::FrameIndex ee_frame_id,
  Eigen::Ref<Eigen::MatrixXd> J_p);

void compute_position_jacobian_time_variation_in_base_frame(
  const pinocchio::Model & model,
  pinocchio::Data & data,
  const Eigen::VectorXd & q,
  const Eigen::VectorXd & v,
  pinocchio::FrameIndex base_frame_id,
  pinocchio::FrameIndex ee_frame_id,
  Eigen::Ref<Eigen::MatrixXd> dJ_p);

Eigen::VectorXd compute_null_space_desired_acceleration(
  const pinocchio::Model & model,
  const Eigen::VectorXd & q,
  const std::vector<double> & q_capture,
  const std::vector<double> & null_space_stiffness,
  const std::vector<int> & arm_configuration_indices,
  const std::vector<int> & arm_velocity_indices);

void fill_joint_acceleration_bounds(
  const std::vector<double> & joint_acceleration_limits,
  const std::vector<int> & arm_velocity_indices,
  Eigen::Ref<Eigen::VectorXd> qdd_min,
  Eigen::Ref<Eigen::VectorXd> qdd_max);

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
  const std::vector<double> & torque_scaling_factors);

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
  TaskImpedanceQpSolver & qp_solver);

}  // namespace pinocchio_task_impedance_controller

#endif  // PINOCCHIO_TASK_IMPEDANCE_CONTROLLER__TASK_IMPEDANCE_CONTROL_LAW_HPP_
