#ifndef PINOCCHIO_TASK_IMPEDANCE_CONTROLLER__TASK_IMPEDANCE_QP_SOLVER_HPP_
#define PINOCCHIO_TASK_IMPEDANCE_CONTROLLER__TASK_IMPEDANCE_QP_SOLVER_HPP_

#include <memory>

#include <Eigen/Core>

namespace pinocchio_task_impedance_controller
{

struct TaskImpedanceQpInput
{
  Eigen::MatrixXd J_p;
  Eigen::Vector3d a_des{Eigen::Vector3d::Zero()};
  Eigen::VectorXd qdd_null;
  double qp_task_weight{1.0};
  double qp_null_weight{1.0};
  Eigen::VectorXd qdd_min;
  Eigen::VectorXd qdd_max;
};

struct TaskImpedanceQpResult
{
  Eigen::VectorXd qdd_d;
  bool success{false};
};

class TaskImpedanceQpSolver
{
public:
  explicit TaskImpedanceQpSolver(int nv);
  ~TaskImpedanceQpSolver();

  TaskImpedanceQpSolver(TaskImpedanceQpSolver && other) noexcept;
  TaskImpedanceQpSolver & operator=(TaskImpedanceQpSolver && other) noexcept;

  TaskImpedanceQpSolver(const TaskImpedanceQpSolver &) = delete;
  TaskImpedanceQpSolver & operator=(const TaskImpedanceQpSolver &) = delete;

  TaskImpedanceQpResult solve(const TaskImpedanceQpInput & input);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace pinocchio_task_impedance_controller

#endif  // PINOCCHIO_TASK_IMPEDANCE_CONTROLLER__TASK_IMPEDANCE_QP_SOLVER_HPP_
