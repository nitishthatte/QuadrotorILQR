#include "src/dynamics.hh"

#include <gtest/gtest.h>

namespace model {
// Demonstrate some basic assertions.
TEST(Dynamics, UpdatesStateCorrectly) {
  const auto x_orig = manif::SE3d::Identity();
  const auto u = manif::SE3d{{1.0, 0.0, 0.0}, manif::SO3d{M_PI, 0.0, 0.0}};

  const auto x_new = dynamics(x_orig, u);
  const auto x_expected = u;

  EXPECT_EQ(x_expected, u);
}
}  // namespace model