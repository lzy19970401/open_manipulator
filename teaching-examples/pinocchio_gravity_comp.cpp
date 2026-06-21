/**
 * Standalone teaching example — NOT part of open_manipulator ROS packages.
 * Pinocchio gravity compensation g(q) for a fixed-base URDF manipulator.
 *
 * Build (ROS 2 workspace with pinocchio installed):
 *   g++ -std=c++17 teaching-examples/pinocchio_gravity_comp.cpp -o /tmp/pinocchio_gc \
 *       $(pkg-config --cflags --libs pinocchio)
 *
 * Run:
 *   /tmp/pinocchio_gc path/to/open_manipulator_x.urdf
 */

#include <iostream>
#include <iomanip>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

int main(int argc, char ** argv)
{
  const std::string urdf_path =
    (argc > 1) ? argv[1] :
    "../open_manipulator_description/urdf/open_manipulator_x/open_manipulator_x.urdf";

  pinocchio::Model model;
  pinocchio::urdf::buildModel(urdf_path, model);
  pinocchio::Data data(model);

  std::cout << "nq=" << model.nq << " nv=" << model.nv << "\n";
  std::cout << "gravity: " << model.gravity.linear().transpose() << "\n";

  Eigen::VectorXd q = pinocchio::neutral(model);
  // SRDF home: joint1..4 = 0, -1, 0.7, 0.3
  if (model.nv >= 4) {
    q[0] = 0.0;
    q[1] = -1.0;
    q[2] = 0.7;
    q[3] = 0.3;
  }

  const Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);
  const Eigen::VectorXd a = Eigen::VectorXd::Zero(model.nv);

  pinocchio::computeGeneralizedGravity(model, data, q);
  const Eigen::VectorXd & g = data.g;

  pinocchio::rnea(model, data, q, v, a);
  const Eigen::VectorXd & tau = data.tau;

  std::cout << std::fixed << std::setprecision(4);
  std::cout << "q      = " << q.head(std::min<int>(4, model.nq)).transpose() << "\n";
  std::cout << "g(q)   = " << g.head(std::min<int>(4, model.nv)).transpose() << "\n";
  std::cout << "rnea   = " << tau.head(std::min<int>(4, model.nv)).transpose() << "\n";
  std::cout << "||g-rnea||_inf = " << (g - tau).lpNorm<Eigen::Infinity>() << "\n";

  return 0;
}
