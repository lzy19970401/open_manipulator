#ifndef PINOCCHIO_IMPEDANCE_CONTROLLER__IMPEDANCE_CONTROL_LAW_HPP_
#define PINOCCHIO_IMPEDANCE_CONTROLLER__IMPEDANCE_CONTROL_LAW_HPP_

#include <om_impedance_utilities/impedance_utilities.hpp>

namespace pinocchio_impedance_controller
{

using om_impedance_utilities::compute_friction_torque;
using om_impedance_utilities::sign_with_deadzone;
using om_impedance_utilities::step_reference_acceleration;
using om_impedance_utilities::step_reference_interpolation;

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

  return om_impedance_utilities::apply_effort_sign_and_scaling(
    tau_phys, effort_sign_flip, torque_scaling);
}

}  // namespace pinocchio_impedance_controller

#endif  // PINOCCHIO_IMPEDANCE_CONTROLLER__IMPEDANCE_CONTROL_LAW_HPP_
