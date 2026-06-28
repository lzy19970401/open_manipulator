#ifndef PINOCCHIO_IMPEDANCE_CONTROLLER__IMPEDANCE_CONTROL_LAW_HPP_
#define PINOCCHIO_IMPEDANCE_CONTROLLER__IMPEDANCE_CONTROL_LAW_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace pinocchio_impedance_controller
{

inline double sign_with_deadzone(double velocity, double deadzone)
{
  if (std::abs(velocity) < deadzone) {
    return 0.0;
  }
  return velocity > 0.0 ? 1.0 : -1.0;
}

inline void step_reference_interpolation(
  std::vector<double> & q_d_active,
  std::vector<double> & q_d_dot_active,
  const std::vector<double> & q_d_target,
  const std::vector<double> & max_velocity,
  double dt)
{
  const size_t n = q_d_active.size();
  const std::vector<double> q_d_prev = q_d_active;

  for (size_t i = 0; i < n; ++i) {
    const double delta = q_d_target[i] - q_d_active[i];
    const double max_step = max_velocity[i] * dt;
    if (std::abs(delta) <= max_step) {
      q_d_active[i] = q_d_target[i];
    } else {
      q_d_active[i] += (delta > 0.0 ? 1.0 : -1.0) * max_step;
    }
  }

  if (dt > 0.0) {
    for (size_t i = 0; i < n; ++i) {
      q_d_dot_active[i] = (q_d_active[i] - q_d_prev[i]) / dt;
    }
  } else {
    std::fill(q_d_dot_active.begin(), q_d_dot_active.end(), 0.0);
  }
}

inline void step_reference_acceleration(
  std::vector<double> & q_d_ddot_active,
  const std::vector<double> & q_d_dot_active,
  const std::vector<double> & q_d_dot_prev,
  double dt)
{
  if (dt > 0.0) {
    for (size_t i = 0; i < q_d_ddot_active.size(); ++i) {
      q_d_ddot_active[i] = (q_d_dot_active[i] - q_d_dot_prev[i]) / dt;
    }
  } else {
    std::fill(q_d_ddot_active.begin(), q_d_ddot_active.end(), 0.0);
  }
}

inline double compute_friction_torque(
  double qdot, double fv, double fc, double deadzone, bool enable_friction)
{
  if (!enable_friction) {
    return 0.0;
  }
  return fv * qdot + fc * sign_with_deadzone(qdot, deadzone);
}

inline double compute_joint_impedance_torque(
  double rnea_torque,
  double q_meas,
  double qdot_meas,
  double q_d,
  double q_d_dot,
  double stiffness,
  double damping,
  double friction_torque,
  double effort_sign_flip,
  double torque_scaling)
{
  const double tau_phys =
    rnea_torque +
    stiffness * (q_d - q_meas) +
    damping * (q_d_dot - qdot_meas) +
    friction_torque;

  return effort_sign_flip * torque_scaling * tau_phys;
}

}  // namespace pinocchio_impedance_controller

#endif  // PINOCCHIO_IMPEDANCE_CONTROLLER__IMPEDANCE_CONTROL_LAW_HPP_
