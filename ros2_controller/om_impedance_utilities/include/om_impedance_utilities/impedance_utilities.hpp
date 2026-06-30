#ifndef OM_IMPEDANCE_UTILITIES__IMPEDANCE_UTILITIES_HPP_
#define OM_IMPEDANCE_UTILITIES__IMPEDANCE_UTILITIES_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace om_impedance_utilities
{

inline double sign_with_deadzone(double velocity, double deadzone)
{
  if (std::abs(velocity) < deadzone) {
    return 0.0;
  }
  return velocity > 0.0 ? 1.0 : -1.0;
}

inline double compute_friction_torque(
  double qdot, double fv, double fc, double deadzone, bool enable_friction)
{
  if (!enable_friction) {
    return 0.0;
  }
  return fv * qdot + fc * sign_with_deadzone(qdot, deadzone);
}

inline double apply_effort_sign_and_scaling(
  double tau_phys, double effort_sign_flip, double torque_scaling)
{
  return effort_sign_flip * torque_scaling * tau_phys;
}

inline void step_reference_interpolation(
  std::vector<double> & reference_active,
  std::vector<double> & reference_dot_active,
  const std::vector<double> & reference_target,
  const std::vector<double> & max_velocity,
  double dt)
{
  const size_t n = reference_active.size();
  const std::vector<double> reference_prev = reference_active;

  for (size_t i = 0; i < n; ++i) {
    const double delta = reference_target[i] - reference_active[i];
    const double max_step = max_velocity[i] * dt;
    if (std::abs(delta) <= max_step) {
      reference_active[i] = reference_target[i];
    } else {
      reference_active[i] += (delta > 0.0 ? 1.0 : -1.0) * max_step;
    }
  }

  if (dt > 0.0) {
    for (size_t i = 0; i < n; ++i) {
      reference_dot_active[i] = (reference_active[i] - reference_prev[i]) / dt;
    }
  } else {
    std::fill(reference_dot_active.begin(), reference_dot_active.end(), 0.0);
  }
}

inline void step_reference_acceleration(
  std::vector<double> & reference_ddot_active,
  const std::vector<double> & reference_dot_active,
  const std::vector<double> & reference_dot_prev,
  double dt)
{
  if (dt > 0.0) {
    for (size_t i = 0; i < reference_ddot_active.size(); ++i) {
      reference_ddot_active[i] = (reference_dot_active[i] - reference_dot_prev[i]) / dt;
    }
  } else {
    std::fill(reference_ddot_active.begin(), reference_ddot_active.end(), 0.0);
  }
}

}  // namespace om_impedance_utilities

#endif  // OM_IMPEDANCE_UTILITIES__IMPEDANCE_UTILITIES_HPP_
