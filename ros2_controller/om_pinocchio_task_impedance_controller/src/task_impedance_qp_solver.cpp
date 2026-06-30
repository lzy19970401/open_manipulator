// Copyright 2026 OpenMANIPULATOR contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include <pinocchio_task_impedance_controller/task_impedance_qp_solver.hpp>

#include <proxsuite/proxqp/dense/dense.hpp>
#include <proxsuite/proxqp/results.hpp>

namespace pinocchio_task_impedance_controller
{

struct TaskImpedanceQpSolver::Impl
{
  explicit Impl(int model_nv)
  : nv(model_nv),
    qp(model_nv, 0, 0, true),
    H(model_nv, model_nv),
    g(model_nv),
    l_box(model_nv),
    u_box(model_nv)
  {
    qp.settings.initial_guess =
      proxsuite::proxqp::InitialGuessStatus::WARM_START_WITH_PREVIOUS_RESULT;
    qp.settings.eps_abs = 1e-8;
    qp.settings.eps_rel = 0.0;
    qp.settings.verbose = false;
    qp.settings.check_duality_gap = false;
  }

  int nv;
  proxsuite::proxqp::dense::QP<double> qp;
  Eigen::MatrixXd H;
  Eigen::VectorXd g;
  Eigen::VectorXd l_box;
  Eigen::VectorXd u_box;
};

TaskImpedanceQpSolver::TaskImpedanceQpSolver(int nv)
: impl_(std::make_unique<Impl>(nv))
{
}

TaskImpedanceQpSolver::~TaskImpedanceQpSolver() = default;

TaskImpedanceQpSolver::TaskImpedanceQpSolver(TaskImpedanceQpSolver && other) noexcept = default;

TaskImpedanceQpSolver & TaskImpedanceQpSolver::operator=(TaskImpedanceQpSolver && other) noexcept =
  default;

TaskImpedanceQpResult TaskImpedanceQpSolver::solve(const TaskImpedanceQpInput & input)
{
  TaskImpedanceQpResult result;
  result.qdd_d.setZero(impl_->nv);

  if (input.J_p.rows() != 3 || input.J_p.cols() != impl_->nv ||
    input.qdd_null.size() != impl_->nv ||
    input.qdd_min.size() != impl_->nv || input.qdd_max.size() != impl_->nv)
  {
    return result;
  }

  const double w_task = input.qp_task_weight;
  const double w_null = input.qp_null_weight;
  const Eigen::Matrix3d W_task = w_task * Eigen::Matrix3d::Identity();
  const Eigen::MatrixXd W_null = w_null * Eigen::MatrixXd::Identity(impl_->nv, impl_->nv);

  impl_->H.noalias() =
    2.0 * (input.J_p.transpose() * W_task * input.J_p + W_null);
  impl_->g.noalias() =
    -2.0 * (input.J_p.transpose() * W_task * input.a_des + W_null * input.qdd_null);

  impl_->l_box = input.qdd_min;
  impl_->u_box = input.qdd_max;

  impl_->qp.init(
    impl_->H, impl_->g, std::nullopt, std::nullopt, std::nullopt, std::nullopt,
    std::nullopt, impl_->l_box, impl_->u_box, false);
  impl_->qp.solve();

  result.success =
    impl_->qp.results.info.status == proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED;
  if (result.success) {
    result.qdd_d = impl_->qp.results.x;
  }
  return result;
}

}  // namespace pinocchio_task_impedance_controller
