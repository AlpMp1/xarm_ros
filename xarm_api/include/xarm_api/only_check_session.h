#ifndef XARM_API_ONLY_CHECK_SESSION_H_
#define XARM_API_ONLY_CHECK_SESSION_H_

#include <cstddef>
#include <functional>
#include <utility>

namespace xarm_api {

struct OnlyCheckStepResult {
  int ret = 0;
  unsigned char only_check_result = 0;
};

struct OnlyCheckPathResult {
  int ret = 0;
  int failing_index = -1;
  unsigned char only_check_result = 0;
};

class OnlyCheckSession {
 public:
  using Setter = std::function<int(unsigned char)>;

  explicit OnlyCheckSession(Setter setter) : setter_(std::move(setter)) {}

  ~OnlyCheckSession() {
    if (reset_required_) {
      setter_(0);
    }
  }

  OnlyCheckSession(const OnlyCheckSession&) = delete;
  OnlyCheckSession& operator=(const OnlyCheckSession&) = delete;

  int SetType(unsigned char type) {
    if (type != 0) {
      reset_required_ = true;
    }
    const int ret = setter_(type);
    if (type == 0 && ret == 0) {
      reset_required_ = false;
    }
    return ret;
  }

 private:
  Setter setter_;
  bool reset_required_ = false;
};

using OnlyCheckStep = std::function<OnlyCheckStepResult(std::size_t)>;

inline OnlyCheckPathResult RunOnlyCheckPath(
    std::size_t waypoint_count,
    OnlyCheckSession::Setter setter,
    OnlyCheckStep check_step) {
  OnlyCheckPathResult result;
  OnlyCheckSession session(std::move(setter));

  for (std::size_t index = 0; index < waypoint_count; ++index) {
    unsigned char type = 3;
    if (index == 0) {
      type = 1;
    } else if (index + 1 == waypoint_count) {
      type = 2;
    }

    const int set_ret = session.SetType(type);
    if (set_ret != 0) {
      result.ret = set_ret;
      result.failing_index = static_cast<int>(index);
      return result;
    }

    const OnlyCheckStepResult step = check_step(index);
    if (step.ret != 0) {
      result.ret = step.ret;
      result.failing_index = static_cast<int>(index);
      result.only_check_result = step.only_check_result;
      return result;
    }
  }
  return result;
}

}  // namespace xarm_api

#endif  // XARM_API_ONLY_CHECK_SESSION_H_
