#include "src/cost.hh"

#include <gtest/gtest.h>
#include <manif/impl/se3/SE3Tangent.h>

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

class ComputeCostDifferentials : public ::testing::Test {
 protected:
  ComputeCostDifferentials()
      : compute_cost_{CostFunction(Q_, R_, {manif::SE3d::Identity()},
                                   {manif::SE3d::Identity()})} {
    auto x_log = manif::SE3Tangentd::Zero();
    x_log.coeffs() = Eigen::Vector<double, 6>{1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
    x_ = x_log.exp();

    auto u_log = manif::SE3Tangentd::Zero();
    u_log.coeffs() = Eigen::Vector<double, 6>{6.0, 5.0, 4.0, 3.0, 2.0, 1.0};
    u_ = u_log.exp();

    compute_cost_(x_, u_, idx_, &cost_diffs_);
  }

  manif::SE3d x_;
  manif::SE3d u_;
  const CostHessianStateState Q_ = CostHessianStateState::Identity();
  const CostHessianControlControl R_ = CostHessianStateState::Identity();
  CostDifferentials cost_diffs_;

  double compute_cost(const manif::SE3d &x, const manif::SE3d &u) {
    return compute_cost_(x, u, idx_);
  }

 private:
  const int idx_ = 0;
  CostFunction compute_cost_;
};

TEST_F(ComputeCostDifferentials, CostStateJacobianCloseToFiniteDifference) {
  const auto EPS = 1e-6;
  CostJacobianState C_x_finite_diff{};
  for (size_t i = 0; i < StateDim; ++i) {
    manif::SE3Tangentd delta_x = manif::SE3Tangentd::Zero();
    delta_x[i] = EPS;
    C_x_finite_diff(i) =
        (compute_cost(x_ + delta_x, u_) - compute_cost(x_ + -1 * delta_x, u_)) /
        (2 * EPS);
  }
  const auto error =
      (cost_diffs_.C_x - C_x_finite_diff).norm() / C_x_finite_diff.size();
  EXPECT_LT(error, EPS);
}

TEST_F(ComputeCostDifferentials, CostStateStateHessianCloseToFiniteDifference) {
  const auto EPS = 1e-6;
  CostHessianStateState C_xx_finite_diff{};
  for (size_t i = 0; i < StateDim; ++i) {
    manif::SE3Tangentd delta_x_0 = manif::SE3Tangentd::Zero();
    delta_x_0[i] = EPS;
    for (size_t j = 0; j < StateDim; ++j) {
      manif::SE3Tangentd delta_x_1 = manif::SE3Tangentd::Zero();
      delta_x_1[j] = EPS;
      C_xx_finite_diff(i, j) =
          (compute_cost(x_ + (delta_x_0 + delta_x_1), u_) -
           compute_cost(x_ + (delta_x_0 - delta_x_1), u_) -
           compute_cost(x_ + (-1 * delta_x_0 + delta_x_1), u_) +
           compute_cost(x_ + (-1 * delta_x_0 - delta_x_1), u_)) /
          (4 * EPS * EPS);
    }
  }

  const auto error = (cost_diffs_.C_xx.inverse() * C_xx_finite_diff -
                      CostHessianStateState::Identity())
                         .norm();
  EXPECT_LT(error, 11.0);
}

TEST_F(ComputeCostDifferentials, CostControlJacobianCloseToFiniteDifference) {
  const auto EPS = 1e-6;
  CostJacobianControl C_u_finite_diff{};
  for (size_t i = 0; i < StateDim; ++i) {
    manif::SE3Tangentd delta_u = manif::SE3Tangentd::Zero();
    delta_u[i] = EPS;
    C_u_finite_diff(i) =
        (compute_cost(x_, u_ + delta_u) - compute_cost(x_, u_ + -1 * delta_u)) /
        (2 * EPS);
  }
  const auto error =
      (cost_diffs_.C_u - C_u_finite_diff).norm() / C_u_finite_diff.size();
  EXPECT_LT(error, EPS);
}

TEST_F(ComputeCostDifferentials,
       CostControlControlHessianCloseToFiniteDifference) {
  const auto EPS = 1e-6;
  CostHessianControlControl C_uu_finite_diff{};
  for (size_t i = 0; i < StateDim; ++i) {
    manif::SE3Tangentd delta_u_0 = manif::SE3Tangentd::Zero();
    delta_u_0[i] = EPS;
    for (size_t j = 0; j < StateDim; ++j) {
      manif::SE3Tangentd delta_u_1 = manif::SE3Tangentd::Zero();
      delta_u_1[j] = EPS;
      C_uu_finite_diff(i, j) =
          (compute_cost(x_, u_ + (delta_u_0 + delta_u_1)) -
           compute_cost(x_, u_ + (delta_u_0 - delta_u_1)) -
           compute_cost(x_, u_ + (-1 * delta_u_0 + delta_u_1)) +
           compute_cost(x_, u_ + (-1 * delta_u_0 - delta_u_1))) /
          (4 * EPS * EPS);
    }
  }

  const auto error = (cost_diffs_.C_uu.inverse() * C_uu_finite_diff -
                      CostHessianControlControl::Identity())
                         .norm();
  EXPECT_LT(error, 110.0);
}
}  // namespace src