#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "xarm_api/only_check_session.h"

namespace {

using xarm_api::OnlyCheckPathResult;
using xarm_api::OnlyCheckStepResult;
using xarm_api::RunOnlyCheckPath;

TEST(OnlyCheckSessionTest, UsesFirstMiddleLastTypesAndAlwaysResets) {
  std::vector<std::string> calls;

  const OnlyCheckPathResult result = RunOnlyCheckPath(
      3,
      [&calls](unsigned char type) {
        calls.push_back("set(" + std::to_string(static_cast<int>(type)) + ")");
        return 0;
      },
      [&calls](std::size_t index) {
        calls.push_back("check(" + std::to_string(index) + ")");
        return OnlyCheckStepResult{};
      });

  EXPECT_EQ(0, result.ret);
  EXPECT_EQ(-1, result.failing_index);
  const std::vector<std::string> expected{
      "set(1)", "check(0)", "set(3)", "check(1)",
      "set(2)", "check(2)", "set(0)"};
  EXPECT_EQ(expected, calls);
}

TEST(OnlyCheckSessionTest, ResetsAfterFirstCheckFails) {
  std::vector<std::string> calls;

  const OnlyCheckPathResult result = RunOnlyCheckPath(
      3,
      [&calls](unsigned char type) {
        calls.push_back("set(" + std::to_string(static_cast<int>(type)) + ")");
        return 0;
      },
      [&calls](std::size_t index) {
        calls.push_back("check(" + std::to_string(index) + ")");
        OnlyCheckStepResult step;
        step.ret = 23;
        step.only_check_result = 7;
        return step;
      });

  EXPECT_EQ(23, result.ret);
  EXPECT_EQ(0, result.failing_index);
  EXPECT_EQ(7, result.only_check_result);
  const std::vector<std::string> expected{"set(1)", "check(0)", "set(0)"};
  EXPECT_EQ(expected, calls);
}

TEST(OnlyCheckSessionTest, ScopeExitResetsAfterActivation) {
  std::vector<int> calls;
  {
    xarm_api::OnlyCheckSession session([&calls](unsigned char type) {
      calls.push_back(type);
      return 0;
    });
    EXPECT_EQ(0, session.SetType(1));
  }

  const std::vector<int> expected{1, 0};
  EXPECT_EQ(expected, calls);
}

TEST(OnlyCheckSessionTest, FailedActivationStillAttemptsReset) {
  std::vector<int> calls;
  bool check_was_called = false;

  const OnlyCheckPathResult result = RunOnlyCheckPath(
      1,
      [&calls](unsigned char type) {
        calls.push_back(type);
        return type == 1 ? 42 : 0;
      },
      [&check_was_called](std::size_t) {
        check_was_called = true;
        return OnlyCheckStepResult{};
      });

  EXPECT_EQ(42, result.ret);
  EXPECT_EQ(0, result.failing_index);
  EXPECT_FALSE(check_was_called);
  const std::vector<int> expected{1, 0};
  EXPECT_EQ(expected, calls);
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
