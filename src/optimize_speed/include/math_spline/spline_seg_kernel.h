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
 * @file : spline_seg_kernel.h
 * @brief: generating integrated kernels for smoothing spline
 *
 *           x' P x  = int_0 ^x  (f(x)^(k))^2 dx, k = 0, 1, 2, 3
 *           P is the kernel of k-th smooth kernel
 **/

#ifndef MATH_SPLINE_SPLINE_SEG_KERNEL_H_
#define MATH_SPLINE_SPLINE_SEG_KERNEL_H_

#include <cstddef>
#include <string>

#include "Eigen/Core"

// #include "modules/common/macro.h"

// A macro to disallow the copy constructor and operator= functions
// This is usually placed in the private: declarations for a class.
#define DISALLOW_COPY_AND_ASSIGN(TypeName) \
  TypeName(const TypeName&) = delete;         \
  void operator=(const TypeName&) = delete

#define DISALLOW_IMPLICIT_CONSTRUCTORS(classname) \
 private:                                         \
  classname();                                    \
  DISALLOW_COPY_AND_ASSIGN(classname);

#define DECLARE_SINGLETON(classname)        \
 public:                                    \
  static classname *instance() {            \
    static classname instance;              \
    return &instance;                       \
  }                                         \
  DISALLOW_IMPLICIT_CONSTRUCTORS(classname) \
 private:

namespace math {
namespace qp_spline {

class SplineSegKernel {
 public:
  // generating kernel matrix
  Eigen::MatrixXd Kernel(const uint32_t num_params, const double accumulated_x);

  // only support N <= 3 cases
  Eigen::MatrixXd NthDerivativeKernel(const uint32_t n,
                                      const uint32_t num_params,
                                      const double accumulated_x);

 private:
  Eigen::MatrixXd DerivativeKernel(const uint32_t num_of_params,
                                   const double accumulated_x);
  Eigen::MatrixXd SecondOrderDerivativeKernel(const uint32_t num_of_params,
                                              const double accumulated_x);
  Eigen::MatrixXd ThirdOrderDerivativeKernel(const uint32_t num_of_params,
                                             const double accumulated_x);

  void IntegratedTermMatrix(const uint32_t num_of_params, const double x,
                            const std::string& type,
                            Eigen::MatrixXd* term_matrix) const;
  void CalculateFx(const uint32_t num_of_params);
  void CalculateDerivative(const uint32_t num_of_params);
  void CalculateSecondOrderDerivative(const uint32_t num_of_params);
  void CalculateThirdOrderDerivative(const uint32_t num_of_params);

  const uint32_t reserved_order_ = 5;
  Eigen::MatrixXd kernel_fx_;
  Eigen::MatrixXd kernel_derivative_;
  Eigen::MatrixXd kernel_second_order_derivative_;
  Eigen::MatrixXd kernel_third_order_derivative_;

  DECLARE_SINGLETON(SplineSegKernel);
};

}  // namespace qp_spline
}  // namespace math

#endif  // MATH_SPLINE_SPLINE_SEG_KERNEL_H_
