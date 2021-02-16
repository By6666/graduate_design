/**
 * @file references.h
 */

#ifndef COMMON_REFERENCES_H_
#define COMMON_REFERENCES_H_

#include <utility>
#include <vector>

#include "reference_point.h"

namespace math {
namespace common {

class References {
 public:
  References() = default;

  void AppendReference(const double x, const double ref);

  const std::vector<ReferencePoint>& reference_points() const;

  double GetReferenceByX(const double x) const;

  void Clear();

 private:
  std::vector<ReferencePoint> reference_points_;
};

}  // namespace common
}  // namespace math

#endif  // COMMON_REFERENCES_H_
