#pragma once

#include <cmath>
#include <cstddef>
#include <sstream>
#include <string>
#include <vector>

namespace xarm_api {

inline bool ValidateFiniteVector(const std::vector<float>& values,
                                 std::size_t expected_size,
                                 const std::string& label,
                                 std::string* reason) {
  if (reason != nullptr) {
    reason->clear();
  }
  if (values.size() != expected_size) {
    if (reason != nullptr) {
      std::ostringstream stream;
      stream << label << " must contain exactly " << expected_size
             << " values";
      *reason = stream.str();
    }
    return false;
  }
  for (float value : values) {
    if (!std::isfinite(value)) {
      if (reason != nullptr) {
        *reason = label + " values must be finite";
      }
      return false;
    }
  }
  return true;
}

}  // namespace xarm_api
