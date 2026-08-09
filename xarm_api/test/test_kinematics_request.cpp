#include <gtest/gtest.h>

#include <limits>
#include <string>
#include <vector>

#include "xarm_api/kinematics_request.h"

namespace {

TEST(KinematicsRequestTest, AcceptsExactFiniteVector) {
  std::string reason;
  EXPECT_TRUE(xarm_api::ValidateFiniteVector(
      std::vector<float>{1, 2, 3, 4, 5, 6}, 6, "joints", &reason));
  EXPECT_TRUE(reason.empty());
}

TEST(KinematicsRequestTest, RejectsWrongValueCount) {
  std::string reason;
  EXPECT_FALSE(xarm_api::ValidateFiniteVector(
      std::vector<float>{1, 2, 3, 4, 5}, 6, "joints", &reason));
  EXPECT_NE(std::string::npos, reason.find("exactly 6"));
}

TEST(KinematicsRequestTest, RejectsNonFiniteValue) {
  std::string reason;
  EXPECT_FALSE(xarm_api::ValidateFiniteVector(
      std::vector<float>{1, 2, 3, 4, 5,
                         std::numeric_limits<float>::quiet_NaN()},
      6, "joints", &reason));
  EXPECT_NE(std::string::npos, reason.find("finite"));
}

TEST(KinematicsRequestTest, AllowsCallerToIgnoreReason) {
  EXPECT_TRUE(xarm_api::ValidateFiniteVector(
      std::vector<float>{1, 2}, 2, "pose", nullptr));
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
