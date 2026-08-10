#ifndef XARM_API_CARTESIAN_PATH_REQUEST_H_
#define XARM_API_CARTESIAN_PATH_REQUEST_H_

#include <array>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

namespace xarm_api {

inline bool ParseCartesianPathRequest(
    const std::vector<float>& flat_poses,
    float velocity,
    float acceleration,
    float move_time,
    float radius,
    std::vector<std::array<float, 6>>* poses,
    std::string* error) {
  if (poses == nullptr || error == nullptr) {
    return false;
  }
  poses->clear();
  error->clear();

  if (flat_poses.empty() || flat_poses.size() % 6 != 0) {
    *error = "poses must be a non-empty row-major multiple of 6";
    return false;
  }
  if (!std::isfinite(velocity) || !std::isfinite(acceleration) ||
      !std::isfinite(move_time) || velocity < 0 || acceleration < 0 ||
      move_time < 0) {
    *error = "mvvelo, mvacc and mvtime must be finite and non-negative";
    return false;
  }
  if (!std::isfinite(radius)) {
    *error = "mvradii must be finite";
    return false;
  }

  poses->reserve(flat_poses.size() / 6);
  for (std::size_t offset = 0; offset < flat_poses.size(); offset += 6) {
    std::array<float, 6> pose{};
    for (std::size_t axis = 0; axis < pose.size(); ++axis) {
      const float value = flat_poses[offset + axis];
      if (!std::isfinite(value)) {
        poses->clear();
        *error = "pose values must be finite";
        return false;
      }
      pose[axis] = value;
    }
    poses->push_back(pose);
  }
  return true;
}

}  // namespace xarm_api

#endif  // XARM_API_CARTESIAN_PATH_REQUEST_H_
