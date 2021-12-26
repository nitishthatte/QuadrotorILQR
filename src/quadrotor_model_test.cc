#include "src/quadrotor_model.hh"

#include <gtest/gtest.h>
#include <manif/impl/se3/SE3Tangent.h>

namespace src {
using State = QuadrotorModel::State;
constexpr auto STATE_DIM = QuadrotorModel::STATE_DIM;
using Control = QuadrotorModel::Control;
constexpr auto CONTROL_DIM = QuadrotorModel::CONTROL_DIM;
using DynamicsDifferentials = QuadrotorModel::DynamicsDifferentials;
constexpr auto dt_s = 0.1;
constexpr auto mass_kg = 1.0;

namespace {
State make_state() {
  return {.inertial_from_body = manif::SE3d::Identity(),
          .body_velocity = manif::SE3Tangentd::Zero()};
}
}  // namespace

TEST(DiscreteDynamics, UpdatesTranslationalStatesCorrectly) {
  auto x_orig = make_state();
  x_orig.body_velocity.lin() = Eigen::Vector3d{1.0, 2.0, 3.0};
  const Control u = Control::Ones();
  const QuadrotorModel quad{.mass_kg_ = mass_kg,
                            .inertia_ = Eigen::Matrix3d::Identity()};

  const auto x_new = quad.discrete_dynamics(x_orig, u, dt_s);

  const Eigen::Vector3d accel_mps =
      (u.sum() / mass_kg - 9.81) * Eigen::Vector3d::UnitZ();
  manif::SE3Tangentd delta_inertial_from_body{};
  delta_inertial_from_body.lin() =
      x_orig.body_velocity.lin() * dt_s + 0.5 * accel_mps * dt_s * dt_s;

  auto x_expected = x_orig;
  x_expected.inertial_from_body =
      x_expected.inertial_from_body + delta_inertial_from_body;
  x_expected.body_velocity.lin() =
      x_orig.body_velocity.lin() + accel_mps * dt_s;

  EXPECT_EQ(x_expected.inertial_from_body, x_new.inertial_from_body);
  EXPECT_EQ(x_expected.body_velocity, x_new.body_velocity);
}

/*
TEST(Dynamics, CalculatesExpectedStateJacobian) {
  const auto x = State::Identity();
  const auto u = Control{{1.0, 0.0, 0.0}, manif::SO3d{M_PI, 0.0, 0.0}};

  DynamicsDifferentials diffs;
  QuadrotorModel::dynamics(x, u, , dt_s, &diffs);

  const State::Jacobian J_x_expected = u.adj().inverse();
  EXPECT_EQ(diffs.J_x, J_x_expected);
}

TEST(Dynamics, StateJacobianCloseToFiniteDifference) {
  const auto x = State::Identity();
  const auto u = Control{{1.0, 0.0, 0.0}, manif::SO3d{M_PI, 0.0, 0.0}};

  DynamicsDifferentials diffs;
  QuadrotorModel::dynamics(x, u, , dt_s, &diffs);

  const auto EPS = 1e-6;
  for (size_t i = 0; i < STATE_DIM; ++i) {
    State::Tangent delta_x = State::Tangent::Zero();
    delta_x[i] = EPS;
    const State::Tangent finit_diff_col =
        (QuadrotorModel::dynamics(x + delta_x, u, dt_s) -
         QuadrotorModel::dynamics(x + -1 * delta_x, u, dt_s)) /
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
  QuadrotorModel::dynamics(x, u, dt_s, &diffs);

  const Control::Jacobian J_u_expected = Control::Jacobian::Identity();
  EXPECT_EQ(diffs.J_u, J_u_expected);
}

TEST(Dynamics, ControlJacobianCloseToFiniteDifference) {
  const auto x = State::Identity();
  const auto u = Control{{1.0, 0.0, 0.0}, manif::SO3d{M_PI, 0.0, 0.0}};

  DynamicsDifferentials diffs;
  QuadrotorModel::dynamics(x, u, dt_s, &diffs);

  const auto EPS = 1e-6;
  for (size_t i = 0; i < CONTROL_DIM; ++i) {
    Control::Tangent delta_u = Control::Tangent::Zero();
    delta_u[i] = EPS;
    const Control::Tangent finit_diff_col =
        (QuadrotorModel::dynamics(x, u + delta_u, dt_s) -
         QuadrotorModel::dynamics(x, u + -1 * delta_u, dt_s)) /
        (2 * EPS);

    const auto error_frac =
        (diffs.J_u.col(i) - finit_diff_col).norm() / (diffs.J_u.col(i).norm());
    EXPECT_LT(error_frac, 0.01);
  }
}
*/
}  // namespace src