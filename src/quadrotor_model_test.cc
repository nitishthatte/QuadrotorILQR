#include "src/quadrotor_model.hh"

#include <gtest/gtest.h>
#include <manif/impl/se3/SE3Tangent.h>

#include <iomanip>
#include <random>

namespace src {
using State = QuadrotorModel::State;
using StateTangent = QuadrotorModel::StateTangent;
using StateBlocks = QuadrotorModel::StateBlocks;
constexpr auto STATE_DIM = QuadrotorModel::STATE_DIM;
constexpr auto CONFIG_DIM = QuadrotorModel::CONFIG_DIM;
using Control = QuadrotorModel::Control;
constexpr auto CONTROL_DIM = QuadrotorModel::CONTROL_DIM;
using DynamicsDifferentials = QuadrotorModel::DynamicsDifferentials;
constexpr auto dt_s = 0.1;
constexpr auto mass_kg = 1.0;

namespace {
Eigen::Matrix3d make_random_inertia_matrix() {
  srand(0u);
  const Eigen::Matrix3d A = Eigen::Matrix3d::Random();
  const Eigen::Matrix3d inertia =
      A * A.transpose() + 3 * Eigen::Matrix3d::Identity();
  return inertia;
}

template <class T>
void check_state_jacobian(const T &fun,
                          const QuadrotorModel::StateJacobian &analytic_diff) {
  constexpr auto EPS = 1e-6;
  for (size_t i = 0; i < STATE_DIM; ++i) {
    auto delta_x = QuadrotorModel::StateTangent::Zero();
    if (i < CONFIG_DIM) {
      delta_x.body_velocity[i] = EPS;
    } else {
      delta_x.body_acceleration[i - CONFIG_DIM] = EPS;
    }

    const auto y_plus = fun(delta_x);
    const auto y_minus = fun(-1 * delta_x);
    const Eigen::Vector<double, STATE_DIM> finite_diff_col =
        ((y_plus - y_minus).coeffs() / (2 * EPS));

    const Eigen::Vector<double, STATE_DIM> &analytic_diff_col =
        analytic_diff.col(i);

    const auto error_rel = (analytic_diff_col - finite_diff_col).norm() /
                           (analytic_diff_col.norm());
    const auto error_abs = (analytic_diff_col - finite_diff_col).norm();
    EXPECT_TRUE(error_rel < 0.01 || error_abs < 1e-12);
  }
}

template <class T>
void check_control_jacobian(
    const T &fun, const QuadrotorModel::ControlJacobian &analytic_diff) {
  constexpr auto EPS = 1e-6;
  for (size_t i = 0; i < CONTROL_DIM; ++i) {
    QuadrotorModel::Control delta_u = QuadrotorModel::Control::Zero();
    delta_u[i] = EPS;

    const auto y_plus = fun(delta_u);
    const auto y_minus = fun(-delta_u);
    const Eigen::Vector<double, STATE_DIM> finite_diff_col =
        ((y_plus - y_minus).coeffs() / (2 * EPS));

    const Eigen::Vector<double, STATE_DIM> &analytic_diff_col =
        analytic_diff.col(i);

    const auto error_rel = (analytic_diff_col - finite_diff_col).norm() /
                           (analytic_diff_col.norm());
    const auto error_abs = (analytic_diff_col - finite_diff_col).norm();
    EXPECT_TRUE(error_rel < 0.01 || error_abs < 1e-12);
  }
}
}  // namespace

class QuadrotorModelTest : public ::testing::Test {
 protected:
  QuadrotorModel quad_{
      mass_kg,
      Eigen::Matrix3d::Identity(),  // inertia
      1.0,                          // arm_length_m
      1.0                           // torque_to_thrust_ratio_m
  };

  State x_init_{.inertial_from_body = manif::SE3d::Identity(),
                .body_velocity = manif::SE3Tangentd::Zero()};
};

TEST_F(QuadrotorModelTest,
       DiscreteDynamicsUpdatesTranslationalStatesCorrectly) {
  x_init_.body_velocity.lin() = Eigen::Vector3d{1.0, 2.0, 3.0};
  const Control u = Control::Ones();

  const auto x_new = quad_.discrete_dynamics(x_init_, u, dt_s);

  const Eigen::Vector3d accel_mpss =
      (u.sum() / mass_kg - 9.81) * Eigen::Vector3d::UnitZ();
  manif::SE3Tangentd delta_inertial_from_body{};
  delta_inertial_from_body.lin() =
      x_init_.body_velocity.lin() * dt_s;  //+ 0.5 * accel_mpss * dt_s * dt_s;

  auto x_expected = x_init_;
  x_expected.inertial_from_body =
      x_expected.inertial_from_body + delta_inertial_from_body;
  x_expected.body_velocity.lin() =
      x_init_.body_velocity.lin() + accel_mpss * dt_s;

  EXPECT_TRUE(x_expected.inertial_from_body.translation().isApprox(
      x_new.inertial_from_body.translation(), 1e-6));
  EXPECT_TRUE(x_expected.body_velocity.isApprox(x_new.body_velocity, 1e-6));
}

TEST_F(QuadrotorModelTest, DiscreteDynamicsUpdatesRotationalStatesCorrectly) {
  x_init_.body_velocity.ang() = Eigen::Vector3d{1.2, 0.0, 0.0};
  const Control u = Eigen::Vector4d{0.0, -1.0, 0.0, 1.0};

  const auto x_new = quad_.discrete_dynamics(x_init_, u, dt_s);

  const Eigen::Vector3d rot_accel_radpss = 2.0 * Eigen::Vector3d::UnitX();
  manif::SE3Tangentd delta_inertial_from_body{};
  delta_inertial_from_body.ang() =
      x_init_.body_velocity.ang() *
      dt_s;  // + 0.5 * rot_accel_radpss * dt_s * dt_s;

  auto x_expected = x_init_;
  x_expected.inertial_from_body =
      x_expected.inertial_from_body + delta_inertial_from_body;
  x_expected.body_velocity.ang() =
      x_init_.body_velocity.ang() + rot_accel_radpss * dt_s;

  EXPECT_LT((x_expected.inertial_from_body.asSO3().inverse() *
             x_new.inertial_from_body.asSO3())
                .log()
                .weightedNorm(),
            1e-6);
  EXPECT_TRUE(
      x_expected.body_velocity.ang().isApprox(x_new.body_velocity.ang(), 1e-6));
}

TEST(QuadrotorDiscreteDynamics, StateJacobianCloseToFiniteDifference) {
  const QuadrotorModel quad{
      mass_kg,
      make_random_inertia_matrix(),  // inertia
      1.0,                           // arm_length_m
      1.0                            // torque_to_thrust_ratio_m
  };
  const State x_init{
      .inertial_from_body =
          manif::SE3Tangentd{
              Eigen::Vector<double, 6>{1.0, 2.0, 3.0, 4.0, 5.0, 6.0}}
              .exp(),
      .body_velocity = Eigen::Vector<double, 6>{2.0, 3.0, 4.0, 5.0, 6.0, 7.0}};
  const auto u = Control::Zero();

  DynamicsDifferentials diffs;
  const auto dt_s = 0.1;
  quad.discrete_dynamics(x_init, u, dt_s, &diffs);

  const auto fun = [&quad, &x_init, &u,
                    dt_s](const QuadrotorModel::StateTangent &delta_x) {
    return quad.discrete_dynamics(x_init + delta_x, u, dt_s);
  };

  check_state_jacobian(fun, diffs.J_x);
}

TEST(QuadrotorDiscreteDynamics, ControlJacobianCloseToFiniteDifference) {
  const QuadrotorModel quad{
      mass_kg,
      make_random_inertia_matrix(),  // inertia
      1.0,                           // arm_length_m
      1.0                            // torque_to_thrust_ratio_m
  };
  const State x_init{
      .inertial_from_body =
          manif::SE3Tangentd{
              Eigen::Vector<double, 6>{1.0, 2.0, 3.0, 4.0, 5.0, 6.0}}
              .exp(),
      .body_velocity = Eigen::Vector<double, 6>{2.0, 3.0, 4.0, 5.0, 6.0, 7.0}};
  const Control u{1.0, 2.0, 3.0, 4.0};

  DynamicsDifferentials diffs;
  const auto dt_s = 0.1;
  quad.discrete_dynamics(x_init, u, dt_s, &diffs);

  const auto fun = [&quad, &x_init, &u,
                    dt_s](const QuadrotorModel::Control &delta_u) {
    return quad.discrete_dynamics(x_init, u + delta_u, dt_s);
  };

  check_control_jacobian(fun, diffs.J_u);
}

TEST(QuadrotorContinuousDynamics, StateJacobianCloseToFiniteDifference) {
  const QuadrotorModel quad{
      mass_kg,
      make_random_inertia_matrix(),  // inertia
      1.0,                           // arm_length_m
      1.0                            // torque_to_thrust_ratio_m
  };
  const State x_init{
      .inertial_from_body =
          manif::SE3Tangentd{
              Eigen::Vector<double, 6>{1.0, 2.0, 3.0, 4.0, 5.0, 6.0}}
              .exp(),
      .body_velocity = Eigen::Vector<double, 6>{2.0, 3.0, 4.0, 5.0, 6.0, 7.0}};
  const auto u = Control::Zero();

  DynamicsDifferentials diffs;
  quad.continuous_dynamics(x_init, u, &diffs);

  const auto fun = [&quad, &x_init,
                    &u](const QuadrotorModel::StateTangent &delta_x) {
    return quad.continuous_dynamics(x_init + delta_x, u);
  };

  check_state_jacobian(fun, diffs.J_x);
}

TEST(QuadrotorContinuousDynamics, ControlJacobianCloseToFiniteDifference) {
  const QuadrotorModel quad{
      mass_kg,
      make_random_inertia_matrix(),  // inertia
      1.0,                           // arm_length_m
      1.0                            // torque_to_thrust_ratio_m
  };
  const State x_init{
      .inertial_from_body =
          manif::SE3Tangentd{
              Eigen::Vector<double, 6>{1.0, 2.0, 3.0, 4.0, 5.0, 6.0}}
              .exp(),
      .body_velocity = Eigen::Vector<double, 6>{2.0, 3.0, 4.0, 5.0, 6.0, 7.0}};
  const Control u{1.0, 2.0, 3.0, 4.0};

  DynamicsDifferentials diffs;
  quad.continuous_dynamics(x_init, u, &diffs);

  const auto fun = [&quad, &x_init,
                    &u](const QuadrotorModel::Control &delta_u) {
    return quad.continuous_dynamics(x_init, u + delta_u);
  };

  check_control_jacobian(fun, diffs.J_u);
}

TEST(StatePlusStateTangentAdd, ComputesCorrectJacobianWrtState) {
  const State x{.inertial_from_body =
                    manif::SE3Tangentd{Eigen::Vector<double, CONFIG_DIM>{
                                           1.0, 2.0, 3.0, 4.0, 5.0, 6.0}}
                        .exp(),
                .body_velocity = Eigen::Vector<double, CONFIG_DIM>{
                    2.0, 3.0, 4.0, 5.0, 6.0, 7.0}};
  const StateTangent tangent{
      .body_velocity = manif::SE3Tangentd{Eigen::Vector<double, CONFIG_DIM>{
          3.0, 4.0, 5.0, 6.0, 7.0, 8.0}},
      .body_acceleration = manif::SE3Tangentd{
          Eigen::Vector<double, CONFIG_DIM>{4.0, 5.0, 6.0, 7.0, 8.0, 9.0}}};

  QuadrotorModel::BinaryStateFuncDiffs diffs;
  add(x, tangent, &diffs);

  const auto fun = [&x, &tangent](const QuadrotorModel::StateTangent &delta_x) {
    return (x + delta_x) + tangent;
  };

  check_state_jacobian(fun, diffs.J_x_lhs);
}

TEST(StatePlusStateTangentAdd, ComputesCorrectJacobianWrtTangent) {
  const State x{.inertial_from_body =
                    manif::SE3Tangentd{Eigen::Vector<double, CONFIG_DIM>{
                                           1.0, 2.0, 3.0, 4.0, 5.0, 6.0}}
                        .exp(),
                .body_velocity = Eigen::Vector<double, CONFIG_DIM>{
                    2.0, 3.0, 4.0, 5.0, 6.0, 7.0}};
  const StateTangent tangent{
      .body_velocity = manif::SE3Tangentd{Eigen::Vector<double, CONFIG_DIM>{
          3.0, 4.0, 5.0, 6.0, 7.0, 8.0}},
      .body_acceleration = manif::SE3Tangentd{
          Eigen::Vector<double, CONFIG_DIM>{4.0, 5.0, 6.0, 7.0, 8.0, 9.0}}};

  QuadrotorModel::BinaryStateFuncDiffs diffs;
  add(x, tangent, &diffs);

  const auto fun =
      [&x, &tangent](const QuadrotorModel::StateTangent &delta_tangent) {
        return x + (tangent + delta_tangent);
      };

  check_state_jacobian(fun, diffs.J_x_rhs);
}

TEST(StateMinusState, ComputesCorrectJacobianWrtLHS) {
  const State lhs{.inertial_from_body =
                      manif::SE3Tangentd{Eigen::Vector<double, CONFIG_DIM>{
                                             1.0, 2.0, 3.0, 4.0, 5.0, 6.0}}
                          .exp(),
                  .body_velocity = Eigen::Vector<double, CONFIG_DIM>{
                      2.0, 3.0, 4.0, 5.0, 6.0, 7.0}};
  const State rhs{.inertial_from_body =
                      manif::SE3Tangentd{2.0 *
                                         Eigen::Vector<double, CONFIG_DIM>{
                                             1.0, 2.0, 3.0, 4.0, 5.0, 6.0}}
                          .exp(),
                  .body_velocity = 2.0 * Eigen::Vector<double, CONFIG_DIM>{
                                             2.0, 3.0, 4.0, 5.0, 6.0, 7.0}};

  QuadrotorModel::BinaryStateFuncDiffs diffs;
  minus(lhs, rhs, &diffs);

  const auto fun = [&lhs, &rhs](const QuadrotorModel::StateTangent &delta_lhs) {
    return (lhs + delta_lhs) - rhs;
  };

  check_state_jacobian(fun, diffs.J_x_lhs);
}

TEST(StateMinusState, ComputesCorrectJacobianWrtRHS) {
  const State lhs{.inertial_from_body =
                      manif::SE3Tangentd{Eigen::Vector<double, CONFIG_DIM>{
                                             1.0, 2.0, 3.0, 4.0, 5.0, 6.0}}
                          .exp(),
                  .body_velocity = Eigen::Vector<double, CONFIG_DIM>{
                      2.0, 3.0, 4.0, 5.0, 6.0, 7.0}};
  const State rhs{.inertial_from_body =
                      manif::SE3Tangentd{2.0 *
                                         Eigen::Vector<double, CONFIG_DIM>{
                                             1.0, 2.0, 3.0, 4.0, 5.0, 6.0}}
                          .exp(),
                  .body_velocity = 2.0 * Eigen::Vector<double, CONFIG_DIM>{
                                             2.0, 3.0, 4.0, 5.0, 6.0, 7.0}};

  QuadrotorModel::BinaryStateFuncDiffs diffs;
  minus(lhs, rhs, &diffs);

  const auto fun = [&lhs, &rhs](const QuadrotorModel::StateTangent &delta_rhs) {
    return lhs - (rhs + delta_rhs);
  };

  check_state_jacobian(fun, diffs.J_x_rhs);
}

TEST(StateTangent, SubscriptOperatorReturnsCorrectCoeffs) {
  const StateTangent tangent{
      .body_velocity = manif::SE3Tangentd{Eigen::Vector<double, CONFIG_DIM>{
          1.0, 2.0, 3.0, 4.0, 5.0, 6.0}},
      .body_acceleration =
          manif::SE3Tangentd{2.0 * Eigen::Vector<double, CONFIG_DIM>{1.0, 2.0,
                                                                     3.0, 4.0,
                                                                     5.0, 6.0}},
  };
  EXPECT_EQ(tangent[0], 1.0);
  EXPECT_EQ(tangent[1], 2.0);
  EXPECT_EQ(tangent[2], 3.0);
  EXPECT_EQ(tangent[3], 4.0);
  EXPECT_EQ(tangent[4], 5.0);
  EXPECT_EQ(tangent[5], 6.0);
  EXPECT_EQ(tangent[6], 2.0);
  EXPECT_EQ(tangent[7], 4.0);
  EXPECT_EQ(tangent[8], 6.0);
  EXPECT_EQ(tangent[9], 8.0);
  EXPECT_EQ(tangent[10], 10.0);
  EXPECT_EQ(tangent[11], 12.0);
}

TEST(StateEquality, ReturnsTrueIfStatesSame) {
  const State lhs{.inertial_from_body =
                      manif::SE3Tangentd{Eigen::Vector<double, CONFIG_DIM>{
                                             1.0, 2.0, 3.0, 4.0, 5.0, 6.0}}
                          .exp(),
                  .body_velocity = Eigen::Vector<double, CONFIG_DIM>{
                      2.0, 3.0, 4.0, 5.0, 6.0, 7.0}};
  EXPECT_EQ(lhs, lhs);
}

TEST(StateEquality, ReturnsFalseIfStatesDifferent) {
  const State lhs{.inertial_from_body =
                      manif::SE3Tangentd{Eigen::Vector<double, CONFIG_DIM>{
                                             1.0, 2.0, 3.0, 4.0, 5.0, 6.0}}
                          .exp(),
                  .body_velocity = Eigen::Vector<double, CONFIG_DIM>{
                      2.0, 3.0, 4.0, 5.0, 6.0, 7.0}};

  const State rhs{.inertial_from_body =
                      manif::SE3Tangentd{Eigen::Vector<double, CONFIG_DIM>{
                                             2.0, 2.0, 3.0, 4.0, 5.0, 6.0}}
                          .exp(),
                  .body_velocity = Eigen::Vector<double, CONFIG_DIM>{
                      2.0, 3.0, 4.0, 5.0, 6.5, 7.0}};

  EXPECT_NE(lhs, rhs);
}

TEST(EulerStep, ComputesCorrectJacobianWrtState) {
  const State x{.inertial_from_body =
                    manif::SE3Tangentd{Eigen::Vector<double, CONFIG_DIM>{
                                           1.0, 2.0, 3.0, 4.0, 5.0, 6.0}}
                        .exp(),
                .body_velocity = Eigen::Vector<double, CONFIG_DIM>{
                    2.0, 3.0, 4.0, 5.0, 6.0, 7.0}};
  const StateTangent x_dot{
      .body_velocity = manif::SE3Tangentd{Eigen::Vector<double, CONFIG_DIM>{
          3.0, 4.0, 5.0, 6.0, 7.0, 8.0}},
      .body_acceleration = manif::SE3Tangentd{
          Eigen::Vector<double, CONFIG_DIM>{4.0, 5.0, 6.0, 7.0, 8.0, 9.0}}};

  QuadrotorModel::BinaryStateFuncDiffs diffs;
  const auto dt_s = 0.1;
  detail::euler_step(x, x_dot, dt_s, &diffs);

  const auto fun = [&x, &x_dot,
                    dt_s](const QuadrotorModel::StateTangent &delta_x) {
    return detail::euler_step(x + delta_x, x_dot, dt_s);
  };

  check_state_jacobian(fun, diffs.J_x_lhs);
}

TEST(EulerStep, ComputesCorrectJacobianWrtStateDeriv) {
  const State x{.inertial_from_body =
                    manif::SE3Tangentd{Eigen::Vector<double, CONFIG_DIM>{
                                           1.0, 2.0, 3.0, 4.0, 5.0, 6.0}}
                        .exp(),
                .body_velocity = Eigen::Vector<double, CONFIG_DIM>{
                    2.0, 3.0, 4.0, 5.0, 6.0, 7.0}};
  const StateTangent x_dot{
      .body_velocity = manif::SE3Tangentd{Eigen::Vector<double, CONFIG_DIM>{
          3.0, 4.0, 5.0, 6.0, 7.0, 8.0}},
      .body_acceleration = manif::SE3Tangentd{
          Eigen::Vector<double, CONFIG_DIM>{4.0, 5.0, 6.0, 7.0, 8.0, 9.0}}};

  QuadrotorModel::BinaryStateFuncDiffs diffs;
  const auto dt_s = 0.1;
  detail::euler_step(x, x_dot, dt_s, &diffs);

  const auto fun = [&x, &x_dot,
                    dt_s](const QuadrotorModel::StateTangent &delta_x_dot) {
    return detail::euler_step(x, x_dot + delta_x_dot, dt_s);
  };

  check_state_jacobian(fun, diffs.J_x_rhs);
}

}  // namespace src