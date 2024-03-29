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
 * @file speed_data.h
 * @brief Speed data base on CommonPoint
 **/

#ifndef PLANNING_COMMON_SPEED_SPEED_DATA_H_
#define PLANNING_COMMON_SPEED_SPEED_DATA_H_

#include <string>
#include <vector>

#include "common/point.h"

namespace math {
namespace qp_spline {

class SpeedData {
 public:
  SpeedData() = default;

  explicit SpeedData(std::vector<CommonPoint> speed_points);

  ~SpeedData() = default;

  const std::vector<CommonPoint>& speed_vector() const;

  void set_speed_vector(std::vector<CommonPoint> speed_points);

  void AppendSpeedPoint(const double t, const double s, const double v, const double a,
                        const double da);

  bool EvaluateByTime(const double time, CommonPoint* const speed_point) const;

  double TotalTime() const;

  bool Empty() const { return speed_vector_.empty(); }

  void Clear();

  void DebugString() const;

 private:
  std::vector<CommonPoint> speed_vector_;
};

}  // namespace qp_spline
}  // namespace math

#endif  // PLANNING_COMMON_SPEED_SPEED_DATA_H_
