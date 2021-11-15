#include "src/cost.hh"

#include <gtest/gtest.h>

namespace src {

using State = LieDynamics::State;
constexpr auto STATE_DIM = LieDynamics::STATE_DIM;
using Control = LieDynamics::Control;
constexpr auto CONTROL_DIM = LieDynamics::CONTROL_DIM;
using CostFunc = CostFunction<LieDynamics>;

TEST(ComputeCost, ReturnsZeroCostWhenZeroError) {
  const auto x = State::Identity();
  const auto u = Control::Identity();
  const CostFunc::CostHessianStateState Q =
      CostFunc::CostHessianStateState::Identity();
  const CostFunc::CostHessianControlControl R =
      CostFunc::CostHessianStateState::Identity();

  const auto compute_cost =
      CostFunc(Q, R, {State::Identity()}, {Control::Identity()});

  const auto cost = compute_cost(x, u, 0);

  EXPECT_EQ(cost, 0.0);
}

class ComputeCostDifferentials : public ::testing::Test {
 protected:
  ComputeCostDifferentials()
      : compute_cost_{
            CostFunc(Q_, R_, {State::Identity()}, {Control::Identity()})} {
    auto x_log = State::Tangent::Zero();
    x_log.coeffs() = Eigen::Vector<double, 6>{1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
    x_ = x_log.exp();

    auto u_log = Control::Tangent::Zero();
    u_log.coeffs() = Eigen::Vector<double, 6>{6.0, 5.0, 4.0, 3.0, 2.0, 1.0};
    u_ = u_log.exp();

    compute_cost_(x_, u_, idx_, &cost_diffs_);
  }

  State x_;
  Control u_;
  const CostFunc::CostHessianStateState Q_ =
      CostFunc::CostHessianStateState::Identity();
  const CostFunc::CostHessianControlControl R_ =
      CostFunc::CostHessianStateState::Identity();
  CostFunc::CostDifferentials cost_diffs_;

  double compute_cost(const State &x, const Control &u) {
    return compute_cost_(x, u, idx_);
  }

 private:
  const int idx_ = 0;
  CostFunc compute_cost_;
};

TEST_F(ComputeCostDifferentials, CostStateJacobianCloseToFiniteDifference) {
  const auto EPS = 1e-6;
  CostFunc::CostJacobianState C_x_finite_diff{};
  for (size_t i = 0; i < STATE_DIM; ++i) {
    State::Tangent delta_x = State::Tangent::Zero();
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
  CostFunc::CostHessianStateState C_xx_finite_diff{};
  for (size_t i = 0; i < STATE_DIM; ++i) {
    State::Tangent delta_x_0 = State::Tangent::Zero();
    delta_x_0[i] = EPS;
    for (size_t j = 0; j < STATE_DIM; ++j) {
      State::Tangent delta_x_1 = State::Tangent::Zero();
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
                      CostFunc::CostHessianStateState::Identity())
                         .norm();
  EXPECT_LT(error, 11.0);
}

TEST_F(ComputeCostDifferentials, CostControlJacobianCloseToFiniteDifference) {
  const auto EPS = 1e-6;
  CostFunc::CostJacobianControl C_u_finite_diff{};
  for (size_t i = 0; i < CONTROL_DIM; ++i) {
    Control::Tangent delta_u = Control::Tangent::Zero();
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
  CostFunc::CostHessianControlControl C_uu_finite_diff{};
  for (size_t i = 0; i < CONTROL_DIM; ++i) {
    Control::Tangent delta_u_0 = Control::Tangent::Zero();
    delta_u_0[i] = EPS;
    for (size_t j = 0; j < CONTROL_DIM; ++j) {
      Control::Tangent delta_u_1 = Control::Tangent::Zero();
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
                      CostFunc::CostHessianControlControl::Identity())
                         .norm();
  EXPECT_LT(error, 110.0);
}
}  // namespace src