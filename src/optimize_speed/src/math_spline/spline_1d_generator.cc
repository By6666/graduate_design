/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file : spline_1d_generator.cc
 * @brief: piecewise_smoothing_spline (pss) generator class
 *           solve pss by qp algorithm, include adding constraint, adding
 *kernel, and solver solve
 **/

#include "math_spline/spline_1d_generator.h"
#include "common/log.h"

#include <algorithm>
#include <ctime>

#include "Eigen/Core"
#include "Eigen/Eigenvalues"

namespace math {
namespace qp_spline {
namespace {

constexpr double kMaxBound = 1e3;
}

using Eigen::MatrixXd;

Spline1dGenerator::Spline1dGenerator(const std::vector<double>& x_knots,
                                     const uint32_t spline_order)
    : spline_(x_knots, spline_order),
      spline_constraint_(x_knots, spline_order),
      spline_kernel_(x_knots, spline_order) {}

void Spline1dGenerator::Reset(const std::vector<double>& x_knots,
                              const uint32_t spline_order) {
  spline_ = Spline1d(x_knots, spline_order);

  spline_constraint_ = Spline1dConstraint(x_knots, spline_order);

  spline_kernel_ = Spline1dKernel(x_knots, spline_order);
}

Spline1dConstraint* Spline1dGenerator::mutable_spline_constraint() {
  return &spline_constraint_;
}

Spline1dKernel* Spline1dGenerator::mutable_spline_kernel() {
  return &spline_kernel_;
}

bool Spline1dGenerator::Solve() {
  const MatrixXd& kernel_matrix = spline_kernel_.kernel_matrix();
  const MatrixXd& offset = spline_kernel_.offset();
  const MatrixXd& inequality_constraint_matrix =
      spline_constraint_.inequality_constraint().constraint_matrix();
  const MatrixXd& inequality_constraint_boundary =
      spline_constraint_.inequality_constraint().constraint_boundary();
  const MatrixXd& equality_constraint_matrix =
      spline_constraint_.equality_constraint().constraint_matrix();
  const MatrixXd& equality_constraint_boundary =
      spline_constraint_.equality_constraint().constraint_boundary();

  if (kernel_matrix.rows() != kernel_matrix.cols()) {
    AERROR << "kernel_matrix.rows() [" << kernel_matrix.rows()
           << "] and kernel_matrix.cols() [" << kernel_matrix.cols()
           << "] should be identical.";
    return false;
  }

  int num_param = kernel_matrix.rows();
  int num_constraint =
      equality_constraint_matrix.rows() + inequality_constraint_matrix.rows();

  bool use_hotstart = last_problem_success_ &&
                      (sqp_solver_ != nullptr && num_param == last_num_param_ &&
                       num_constraint == last_num_constraint_);

  if (!use_hotstart) {
    sqp_solver_.reset(new ::qpOASES::SQProblem(num_param, num_constraint,
                                               ::qpOASES::HST_UNKNOWN));
    ::qpOASES::Options my_options;
    my_options.enableCholeskyRefactorisation = 1;
    my_options.epsNum = -1e-7;
    my_options.epsDen = 1e-7;
    my_options.epsIterRef = 1e-7;
    sqp_solver_->setOptions(my_options);
    if (true) {
      sqp_solver_->setPrintLevel(qpOASES::PL_NONE);
    }
  }

  // definition of qpOASESproblem
  const int kNumOfMatrixElements = kernel_matrix.rows() * kernel_matrix.cols();
  double h_matrix[kNumOfMatrixElements];  // NOLINT

  const int kNumOfOffsetRows = offset.rows();
  double g_matrix[kNumOfOffsetRows];  // NOLINT
  int index = 0;

  for (int r = 0; r < kernel_matrix.rows(); ++r) {
    g_matrix[r] = offset(r, 0);
    for (int c = 0; c < kernel_matrix.cols(); ++c) {
      h_matrix[index++] = kernel_matrix(r, c);
    }
  }
  DCHECK_EQ(index, kernel_matrix.rows() * kernel_matrix.cols());

  // search space lower bound and uppper bound
  double lower_bound[num_param];  // NOLINT
  double upper_bound[num_param];  // NOLINT

  const double l_lower_bound_ = -kMaxBound;
  const double l_upper_bound_ = kMaxBound;
  for (int i = 0; i < num_param; ++i) {
    lower_bound[i] = l_lower_bound_;
    upper_bound[i] = l_upper_bound_;
  }

  // constraint matrix construction
  double affine_constraint_matrix[num_param * num_constraint];  // NOLINT
  double constraint_lower_bound[num_constraint];                // NOLINT
  double constraint_upper_bound[num_constraint];                // NOLINT

  index = 0;
  for (int r = 0; r < equality_constraint_matrix.rows(); ++r) {
    constraint_lower_bound[r] = equality_constraint_boundary(r, 0);
    constraint_upper_bound[r] = equality_constraint_boundary(r, 0);

    for (int c = 0; c < num_param; ++c) {
      affine_constraint_matrix[index++] = equality_constraint_matrix(r, c);
    }
  }

  DCHECK_EQ(index, equality_constraint_matrix.rows() * num_param);

  const double constraint_upper_bound_ = kMaxBound;
  for (int r = 0; r < inequality_constraint_matrix.rows(); ++r) {
    constraint_lower_bound[r + equality_constraint_boundary.rows()] =
        inequality_constraint_boundary(r, 0);
    constraint_upper_bound[r + equality_constraint_boundary.rows()] =
        constraint_upper_bound_;

    for (int c = 0; c < num_param; ++c) {
      affine_constraint_matrix[index++] = inequality_constraint_matrix(r, c);
    }
  }
  DCHECK_EQ(index, equality_constraint_matrix.rows() * num_param +
                       inequality_constraint_boundary.rows() * num_param);

  // initialize problem
  int max_iteration_ = 1000;
  int max_iter = std::max(max_iteration_, num_constraint);

  ::qpOASES::returnValue ret;
  auto start_timestamp = clock();
  if (use_hotstart) {
    ADEBUG << "using SQP hotstart.";
    ret = sqp_solver_->hotstart(
        h_matrix, g_matrix, affine_constraint_matrix, lower_bound, upper_bound,
        constraint_lower_bound, constraint_upper_bound, max_iter);
    if (ret != qpOASES::SUCCESSFUL_RETURN) {
      ADEBUG << "Fail to hotstart spline 1d, will use re-init instead.";
      ret = sqp_solver_->init(h_matrix, g_matrix, affine_constraint_matrix,
                              lower_bound, upper_bound, constraint_lower_bound,
                              constraint_upper_bound, max_iter);
    }
  } else {
    ADEBUG << "no using SQP hotstart.";
    ret = sqp_solver_->init(h_matrix, g_matrix, affine_constraint_matrix,
                            lower_bound, upper_bound, constraint_lower_bound,
                            constraint_upper_bound, max_iter);
  }
  auto end_timestamp = clock();
  ADEBUG << "Spline1dGenerator QP solve time: "
         << (end_timestamp - start_timestamp) / 1000.0 << " ms.";

  if (ret != qpOASES::SUCCESSFUL_RETURN) {
    if (ret == qpOASES::RET_MAX_NWSR_REACHED) {
      ADEBUG << "qpOASES solver failed due to reached max iteration";
    } else {
      ADEBUG << "qpOASES solver failed due to infeasibility or other internal "
                "reasons:"
             << ret;
    }
    last_problem_success_ = false;
    return false;
  }

  last_problem_success_ = true;
  double result[num_param];  // NOLINT
  memset(result, 0, sizeof result);

  sqp_solver_->getPrimalSolution(result);

  MatrixXd solved_params = MatrixXd::Zero(num_param, 1);
  for (int i = 0; i < num_param; ++i) {
    solved_params(i, 0) = result[i];
    ADEBUG << "spline 1d solved param[" << i << "]: " << result[i];
  }

  last_num_param_ = num_param;
  last_num_constraint_ = num_constraint;

  return spline_.SetSplineSegs(solved_params, spline_.spline_order());
}

const Spline1d& Spline1dGenerator::spline() const { return spline_; }

}  // namespace qp_spline
}  // namespace math
