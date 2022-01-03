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
  const auto EPS = 1e-6;
  for (size_t i = 0; i < STATE_DIM; ++i) {
    StateTangent delta_x{.body_velocity = manif::SE3Tangentd::Zero(),
                         .body_acceleration = manif::SE3Tangentd::Zero()};
    if (i < CONFIG_DIM) {
      delta_x.body_velocity[i] = EPS;
    } else {
      delta_x.body_acceleration[i - CONFIG_DIM] = EPS;
    }

    const auto EPS = 1e-6;
    const auto x_plus = fun(delta_x);
    const auto x_minus = fun(-1 * delta_x);
    const Eigen::Vector<double, STATE_DIM> finite_diff_col =
        ((x_plus - x_minus).coeffs() / (2 * EPS));

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
      x_init_.body_velocity.lin() * dt_s + 0.5 * accel_mpss * dt_s * dt_s;

  auto x_expected = x_init_;
  x_expected.inertial_from_body =
      x_expected.inertial_from_body + delta_inertial_from_body;
  x_expected.body_velocity.lin() =
      x_init_.body_velocity.lin() + accel_mpss * dt_s;

  EXPECT_EQ(x_expected.inertial_from_body, x_new.inertial_from_body);
  EXPECT_EQ(x_expected.body_velocity, x_new.body_velocity);
}

TEST_F(QuadrotorModelTest, DiscreteDynamicsUpdatesRotationalStatesCorrectly) {
  x_init_.body_velocity.ang() = Eigen::Vector3d{1.2, 0.0, 0.0};
  const Control u = Eigen::Vector4d{0.0, -1.0, 0.0, 1.0};

  const auto x_new = quad_.discrete_dynamics(x_init_, u, dt_s);

  const Eigen::Vector3d rot_accel_radpss = 2.0 * Eigen::Vector3d::UnitX();
  manif::SE3Tangentd delta_inertial_from_body{};
  delta_inertial_from_body.ang() =
      x_init_.body_velocity.ang() * dt_s + 0.5 * rot_accel_radpss * dt_s * dt_s;

  auto x_expected = x_init_;
  x_expected.inertial_from_body =
      x_expected.inertial_from_body + delta_inertial_from_body;
  x_expected.body_velocity.ang() =
      x_init_.body_velocity.ang() + rot_accel_radpss * dt_s;

  EXPECT_EQ(x_expected.inertial_from_body.rotation(),
            x_new.inertial_from_body.rotation());
  EXPECT_EQ(x_expected.body_velocity.ang(), x_new.body_velocity.ang());
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
                    u](const QuadrotorModel::StateTangent &delta_x) {
    return quad.continuous_dynamics(x_init + delta_x, u);
  };

  check_state_jacobian(fun, diffs.J_x);
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

  QuadrotorModel::StateJacobian J_x;
  QuadrotorModel::StateJacobian J_t;
  add(x, tangent, &J_x, &J_t);

  const auto fun = [&x, &tangent](const QuadrotorModel::StateTangent &delta_x) {
    return (x + delta_x) + tangent;
  };

  check_state_jacobian(fun, J_x);
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

  QuadrotorModel::StateJacobian J_x;
  QuadrotorModel::StateJacobian J_t;
  add(x, tangent, &J_x, &J_t);

  const auto fun = [&x, &tangent](const QuadrotorModel::StateTangent &delta_x) {
    return x + (tangent + delta_x);
  };

  check_state_jacobian(fun, J_t);
}

/*
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