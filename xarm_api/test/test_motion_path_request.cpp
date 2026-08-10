#include <gtest/gtest.h>

#include <limits>
#include <string>
#include <vector>

#include "xarm_api/motion_path_request.h"

namespace {

TEST(MotionPathRequestTest, ParsesExactMixedPrimitiveOrder) {
  std::vector<xarm_api::MotionPathRequestSegment> input(3);
  input[0].motion_type = xarm_api::MotionPrimitive::kCartesian;
  input[0].target = {300, 0, 300, 3.14F, 0, 0};
  input[0].velocity = 380;
  input[0].acceleration = 1100;
  input[0].radius = 20;
  input[1].motion_type = xarm_api::MotionPrimitive::kJoint;
  input[1].target = {0, 0.1F, 0.2F, 0.3F, 0.4F, 0.5F};
  input[1].velocity = 1.0F;
  input[1].acceleration = 3.0F;
  input[2] = input[0];
  input[2].target[0] = 420;

  std::vector<xarm_api::MotionPathSegment> parsed;
  std::string error;
  ASSERT_TRUE(xarm_api::ParseMotionPathRequest(input, 6, &parsed, &error));
  ASSERT_EQ(3u, parsed.size());
  EXPECT_EQ(xarm_api::MotionPrimitive::kCartesian,
            parsed[0].motion_type);
  EXPECT_EQ(xarm_api::MotionPrimitive::kJoint, parsed[1].motion_type);
  EXPECT_FLOAT_EQ(0.5F, parsed[1].target[5]);
  EXPECT_FLOAT_EQ(420.0F, parsed[2].target[0]);
  EXPECT_TRUE(error.empty());
}

TEST(MotionPathRequestTest, RejectsMalformedSegmentsBeforeControllerUse) {
  const float nan = std::numeric_limits<float>::quiet_NaN();
  xarm_api::MotionPathRequestSegment valid;
  valid.motion_type = xarm_api::MotionPrimitive::kCartesian;
  valid.target = {300, 0, 300, 3.14F, 0, 0};
  valid.velocity = 380;
  valid.acceleration = 1100;
  valid.radius = 20;

  struct Case {
    std::vector<xarm_api::MotionPathRequestSegment> segments;
    std::size_t dof;
    const char* error;
  };
  xarm_api::MotionPathRequestSegment bad_type = valid;
  bad_type.motion_type = static_cast<xarm_api::MotionPrimitive>(9);
  xarm_api::MotionPathRequestSegment bad_cartesian_size = valid;
  bad_cartesian_size.target.pop_back();
  xarm_api::MotionPathRequestSegment bad_joint_size = valid;
  bad_joint_size.motion_type = xarm_api::MotionPrimitive::kJoint;
  bad_joint_size.target.pop_back();
  xarm_api::MotionPathRequestSegment bad_value = valid;
  bad_value.target[2] = nan;
  xarm_api::MotionPathRequestSegment bad_speed = valid;
  bad_speed.velocity = -1;

  const std::vector<Case> cases{
      {{}, 6, "motion path must contain at least one segment"},
      {{valid}, 0, "robot DOF must be in [1, 7]"},
      {{bad_type}, 6, "motion segment 0 has an unknown primitive"},
      {{bad_cartesian_size}, 6,
       "Cartesian motion segment 0 must contain exactly 6 values"},
      {{bad_joint_size}, 6,
       "joint motion segment 0 must match the robot DOF"},
      {{bad_value}, 6, "motion segment 0 contains non-finite values"},
      {{bad_speed}, 6,
       "motion segment 0 parameters must be finite and non-negative"},
  };

  for (const Case& test_case : cases) {
    std::vector<xarm_api::MotionPathSegment> parsed;
    std::string error;
    EXPECT_FALSE(xarm_api::ParseMotionPathRequest(
        test_case.segments, test_case.dof, &parsed, &error));
    EXPECT_EQ(test_case.error, error);
    EXPECT_TRUE(parsed.empty());
  }
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
