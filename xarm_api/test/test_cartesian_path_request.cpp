#include <gtest/gtest.h>

#include <array>
#include <limits>
#include <string>
#include <vector>

#include "xarm_api/cartesian_path_request.h"

namespace {

TEST(CartesianPathRequestTest, ParsesTwoFinitePoses) {
  const std::vector<float> flat{1, 2, 3, 4, 5, 6,
                                7, 8, 9, 10, 11, 12};
  std::vector<std::array<float, 6>> poses;
  std::string error;

  EXPECT_TRUE(xarm_api::ParseCartesianPathRequest(
      flat, 380.0F, 1100.0F, 0.0F, 20.0F, &poses, &error));
  ASSERT_EQ(2u, poses.size());
  EXPECT_FLOAT_EQ(12.0F, poses[1][5]);
  EXPECT_TRUE(error.empty());
}

TEST(CartesianPathRequestTest, RejectsInvalidRequestsWithSpecificReasons) {
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const std::vector<float> valid{1, 2, 3, 4, 5, 6};

  struct Case {
    std::vector<float> poses;
    float speed;
    float acceleration;
    float move_time;
    float radius;
    const char* expected_error;
  };
  const std::vector<Case> cases{
      {{}, 1, 1, 0, 0, "poses must be a non-empty row-major multiple of 6"},
      {{1, 2, 3}, 1, 1, 0, 0,
       "poses must be a non-empty row-major multiple of 6"},
      {{1, 2, 3, 4, 5, nan}, 1, 1, 0, 0,
       "pose values must be finite"},
      {valid, -1, 1, 0, 0,
       "mvvelo, mvacc and mvtime must be finite and non-negative"},
      {valid, 1, -1, 0, 0,
       "mvvelo, mvacc and mvtime must be finite and non-negative"},
      {valid, 1, 1, nan, 0,
       "mvvelo, mvacc and mvtime must be finite and non-negative"},
      {valid, 1, 1, 0, nan, "mvradii must be finite"},
  };

  for (const Case& test_case : cases) {
    std::vector<std::array<float, 6>> poses;
    std::string error;
    EXPECT_FALSE(xarm_api::ParseCartesianPathRequest(
        test_case.poses, test_case.speed, test_case.acceleration,
        test_case.move_time, test_case.radius, &poses, &error));
    EXPECT_EQ(test_case.expected_error, error);
    EXPECT_TRUE(poses.empty());
  }
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
