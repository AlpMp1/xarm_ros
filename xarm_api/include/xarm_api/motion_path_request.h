#ifndef XARM_API_MOTION_PATH_REQUEST_H_
#define XARM_API_MOTION_PATH_REQUEST_H_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

namespace xarm_api {

enum class MotionPrimitive : unsigned char {
  kCartesian = 0,
  kJoint = 1,
};

struct MotionPathRequestSegment {
  MotionPrimitive motion_type = MotionPrimitive::kCartesian;
  std::vector<float> target;
  float velocity = 0;
  float acceleration = 0;
  float move_time = 0;
  float radius = 0;
};

struct MotionPathSegment {
  MotionPrimitive motion_type = MotionPrimitive::kCartesian;
  std::array<float, 7> target{};
  std::size_t target_size = 0;
  float velocity = 0;
  float acceleration = 0;
  float move_time = 0;
  float radius = 0;
};

inline bool ParseMotionPathRequest(
    const std::vector<MotionPathRequestSegment>& request,
    std::size_t robot_dof,
    std::vector<MotionPathSegment>* parsed,
    std::string* error) {
  if (parsed == nullptr || error == nullptr) {
    return false;
  }
  parsed->clear();
  error->clear();
  if (request.empty()) {
    *error = "motion path must contain at least one segment";
    return false;
  }
  if (robot_dof == 0 || robot_dof > 7) {
    *error = "robot DOF must be in [1, 7]";
    return false;
  }

  parsed->reserve(request.size());
  for (std::size_t index = 0; index < request.size(); ++index) {
    const MotionPathRequestSegment& source = request[index];
    if (source.motion_type != MotionPrimitive::kCartesian &&
        source.motion_type != MotionPrimitive::kJoint) {
      *error = "motion segment " + std::to_string(index) +
               " has an unknown primitive";
      parsed->clear();
      return false;
    }
    const std::size_t expected_size =
        source.motion_type == MotionPrimitive::kCartesian ? 6 : robot_dof;
    if (source.target.size() != expected_size) {
      *error = source.motion_type == MotionPrimitive::kCartesian
                   ? "Cartesian motion segment " + std::to_string(index) +
                         " must contain exactly 6 values"
                   : "joint motion segment " + std::to_string(index) +
                         " must match the robot DOF";
      parsed->clear();
      return false;
    }
    if (!std::all_of(source.target.begin(), source.target.end(),
                     [](float value) { return std::isfinite(value); })) {
      *error = "motion segment " + std::to_string(index) +
               " contains non-finite values";
      parsed->clear();
      return false;
    }
    if (!std::isfinite(source.velocity) ||
        !std::isfinite(source.acceleration) ||
        !std::isfinite(source.move_time) || !std::isfinite(source.radius) ||
        source.velocity < 0 || source.acceleration < 0 ||
        source.move_time < 0) {
      *error = "motion segment " + std::to_string(index) +
               " parameters must be finite and non-negative";
      parsed->clear();
      return false;
    }

    MotionPathSegment destination;
    destination.motion_type = source.motion_type;
    destination.target_size = source.target.size();
    std::copy(source.target.begin(), source.target.end(),
              destination.target.begin());
    destination.velocity = source.velocity;
    destination.acceleration = source.acceleration;
    destination.move_time = source.move_time;
    destination.radius = source.radius;
    parsed->push_back(destination);
  }
  return true;
}

}  // namespace xarm_api

#endif  // XARM_API_MOTION_PATH_REQUEST_H_
