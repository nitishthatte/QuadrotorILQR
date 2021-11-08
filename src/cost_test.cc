#include "src/cost.hh"

#include <gtest/gtest.h>

namespace src {
TEST(ComputeCost, ReturnsZeroCostWhenZeroError) {
  const auto x = manif::SE3d::Identity();
  const auto u = manif::SE3d::Identity();
  const CostHessianStateState Q = CostHessianStateState::Identity();
  const CostHessianControlControl R = CostHessianStateState::Identity();

  const auto compute_cost =
      CostFunction(Q, R, {manif::SE3d::Identity()}, {manif::SE3d::Identity()});

  const auto cost = compute_cost(x, u, 0);

  EXPECT_EQ(cost, 0.0);
}
}  // namespace src