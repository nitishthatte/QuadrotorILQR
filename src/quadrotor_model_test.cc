#include "src/quadrotor_model.hh"

#include <gtest/gtest.h>
#include <manif/impl/se3/SE3Tangent.h>

#include <iomanip>

namespace src {
using State = QuadrotorModel::State;
constexpr auto STATE_DIM = QuadrotorModel::STATE_DIM;
using Control = QuadrotorModel::Control;
constexpr auto CONTROL_DIM = QuadrotorModel::CONTROL_DIM;
using DynamicsDifferentials = QuadrotorModel::DynamicsDifferentials;
constexpr auto dt_s = 0.1;
constexpr auto mass_kg = 1.0;

class QuadrotorModelTest : public ::testing::Test {
 protected:
  const QuadrotorModel quad_{.mass_kg_ = mass_kg,
                             .inertia_ = Eigen::Matrix3d::Identity(),
                             .arm_length_m_ = 1.0,
                             .torque_to_thrust_ratio_m_ = 1.0};
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

TEST_F(QuadrotorModelTest,
       ContinuousDynamicsStateJacobianCloseToFiniteDifference) {
  const auto u = Control::Zero();

  DynamicsDifferentials diffs;
  quad_.continuous_dynamics(x_init_, u, &diffs);

  const auto EPS = 1e-6;
  for (size_t i = 0; i < 3; ++i) {
    QuadrotorModel::StateTangent delta_x{
        .body_velocity = manif::SE3Tangentd::Zero(),
        .body_acceleration = manif::SE3Tangentd::Zero()};
    delta_x.body_velocity.ang()[i] = EPS;

    const auto x_plus = quad_.continuous_dynamics(x_init_ + delta_x, u);
    const auto x_minus = quad_.continuous_dynamics(x_init_ - delta_x, u);
    const Eigen::Vector<double, 3> finite_diff_vel_rot_i =
        ((x_plus - x_minus).coeffs() /
         (2 * EPS))(QuadrotorModel::StateBlocks::body_lin_vel);

    const Eigen::Vector<double, 3> &analytic_diff_vel_rot_i =
        diffs.J_x(QuadrotorModel::StateBlocks::body_lin_vel,
                  QuadrotorModel::StateBlocks::inertial_from_body_rot[i]);

    const auto error_rel =
        (analytic_diff_vel_rot_i - finite_diff_vel_rot_i).norm() /
        (analytic_diff_vel_rot_i.norm());
    const auto error_abs =
        (analytic_diff_vel_rot_i - finite_diff_vel_rot_i).norm();
    EXPECT_TRUE(error_rel < 0.01 || error_abs < 1e-12);
  }
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