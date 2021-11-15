#include "src/dynamics.hh"

#include <gtest/gtest.h>
#include <manif/impl/se3/SE3.h>
#include <manif/impl/se3/SE3Tangent.h>

namespace src {
using State = LieDynamics::State;
constexpr auto STATE_DIM = LieDynamics::STATE_DIM;
using Control = LieDynamics::Control;
constexpr auto CONTROL_DIM = LieDynamics::CONTROL_DIM;
using DynamicsDifferentials = LieDynamics::DynamicsDifferentials;

TEST(Dynamics, UpdatesStateCorrectly) {
  const auto x_orig = State::Identity();
  const auto u = Control{{1.0, 0.0, 0.0}, manif::SO3d{M_PI, 0.0, 0.0}};

  const auto x_new = LieDynamics::dynamics(x_orig, u);
  const auto x_expected = u;

  EXPECT_EQ(x_expected, x_new);
}

TEST(Dynamics, CalculatesExpectedStateJacobian) {
  const auto x = State::Identity();
  const auto u = Control{{1.0, 0.0, 0.0}, manif::SO3d{M_PI, 0.0, 0.0}};

  DynamicsDifferentials diffs;
  LieDynamics::dynamics(x, u, &diffs);

  const State::Jacobian J_x_expected = u.adj().inverse();
  EXPECT_EQ(diffs.J_x, J_x_expected);
}

TEST(Dynamics, StateJacobianCloseToFiniteDifference) {
  const auto x = State::Identity();
  const auto u = Control{{1.0, 0.0, 0.0}, manif::SO3d{M_PI, 0.0, 0.0}};

  DynamicsDifferentials diffs;
  LieDynamics::dynamics(x, u, &diffs);

  const auto EPS = 1e-6;
  for (size_t i = 0; i < STATE_DIM; ++i) {
    State::Tangent delta_x = State::Tangent::Zero();
    delta_x[i] = EPS;
    const State::Tangent finit_diff_col =
        (LieDynamics::dynamics(x + delta_x, u) -
         LieDynamics::dynamics(x + -1 * delta_x, u)) /
        (2 * EPS);

    const auto error_frac =
        (diffs.J_x.col(i) - finit_diff_col).norm() / (diffs.J_x.col(i).norm());
    EXPECT_LT(error_frac, 0.01);
  }
}

TEST(Dynamics, CalculatesExpectedControlJacobian) {
  const auto x = State::Identity();
  const auto u = Control{{1.0, 0.0, 0.0}, manif::SO3d{M_PI, 0.0, 0.0}};

  DynamicsDifferentials diffs;
  LieDynamics::dynamics(x, u, &diffs);

  const Control::Jacobian J_u_expected = Control::Jacobian::Identity();
  EXPECT_EQ(diffs.J_u, J_u_expected);
}

TEST(Dynamics, ControlJacobianCloseToFiniteDifference) {
  const auto x = State::Identity();
  const auto u = Control{{1.0, 0.0, 0.0}, manif::SO3d{M_PI, 0.0, 0.0}};

  DynamicsDifferentials diffs;
  LieDynamics::dynamics(x, u, &diffs);

  const auto EPS = 1e-6;
  for (size_t i = 0; i < CONTROL_DIM; ++i) {
    Control::Tangent delta_u = Control::Tangent::Zero();
    delta_u[i] = EPS;
    const Control::Tangent finit_diff_col =
        (LieDynamics::dynamics(x, u + delta_u) -
         LieDynamics::dynamics(x, u + -1 * delta_u)) /
        (2 * EPS);

    const auto error_frac =
        (diffs.J_u.col(i) - finit_diff_col).norm() / (diffs.J_u.col(i).norm());
    EXPECT_LT(error_frac, 0.01);
  }
}
}  // namespace src