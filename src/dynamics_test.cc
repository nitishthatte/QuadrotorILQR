#include "src/dynamics.hh"

#include <gtest/gtest.h>
#include <manif/impl/se3/SE3.h>
#include <manif/impl/se3/SE3Tangent.h>

namespace src {
TEST(Dynamics, UpdatesStateCorrectly) {
  const auto x_orig = manif::SE3d::Identity();
  const auto u = manif::SE3d{{1.0, 0.0, 0.0}, manif::SO3d{M_PI, 0.0, 0.0}};

  const auto x_new = dynamics(x_orig, u);
  const auto x_expected = u;

  EXPECT_EQ(x_expected, x_new);
}

TEST(Dynamics, CalculatesExpectedStateJacobian) {
  const auto x_orig = manif::SE3d::Identity();
  const auto u = manif::SE3d{{1.0, 0.0, 0.0}, manif::SO3d{M_PI, 0.0, 0.0}};

  manif::SE3d::Jacobian J_x = manif::SE3d::Jacobian::Zero();
  dynamics(x_orig, u, &J_x);

  const manif::SE3d::Jacobian J_x_expected = u.adj().inverse();
  EXPECT_EQ(J_x, J_x_expected);
}

TEST(Dynamics, StateJacobianCloseToFiniteDifference) {
  const auto x_orig = manif::SE3d::Identity();
  const auto u = manif::SE3d{{1.0, 0.0, 0.0}, manif::SO3d{M_PI, 0.0, 0.0}};

  manif::SE3d::Jacobian J_x = manif::SE3d::Jacobian::Zero();
  dynamics(x_orig, u, &J_x);

  const auto EPS = 1e-6;
  for (size_t i = 0; i < StateDim; ++i) {
    manif::SE3Tangentd delta_x = manif::SE3Tangentd::Zero();
    delta_x[i] = EPS;
    const manif::SE3Tangentd finit_diff_col =
        ((x_orig + delta_x) * u - (x_orig + -1 * delta_x) * u) / (2 * EPS);

    const auto error_frac =
        (J_x.col(i) - finit_diff_col).norm() / (J_x.col(i).norm());
    EXPECT_LT(error_frac, 0.01);
  }
}

TEST(Dynamics, CalculatesExpectedControlJacobian) {
  const auto x_orig = manif::SE3d::Identity();
  const auto u = manif::SE3d{{1.0, 0.0, 0.0}, manif::SO3d{M_PI, 0.0, 0.0}};

  manif::SE3d::Jacobian J_u = manif::SE3d::Jacobian::Zero();
  dynamics(x_orig, u, nullptr, &J_u);

  const manif::SE3d::Jacobian J_u_expected = manif::SE3d::Jacobian::Identity();
  EXPECT_EQ(J_u, J_u_expected);
}

TEST(Dynamics, ControlJacobianCloseToFiniteDifference) {
  const auto x_orig = manif::SE3d::Identity();
  const auto u = manif::SE3d{{1.0, 0.0, 0.0}, manif::SO3d{M_PI, 0.0, 0.0}};

  manif::SE3d::Jacobian J_u = manif::SE3d::Jacobian::Zero();
  dynamics(x_orig, u, nullptr, &J_u);

  const auto EPS = 1e-6;
  for (size_t i = 0; i < StateDim; ++i) {
    manif::SE3Tangentd delta_u = manif::SE3Tangentd::Zero();
    delta_u[i] = EPS;
    const manif::SE3Tangentd finit_diff_col =
        (x_orig * (u + delta_u) - (x_orig * (u + -1 * delta_u))) / (2 * EPS);

    const auto error_frac =
        (J_u.col(i) - finit_diff_col).norm() / (J_u.col(i).norm());
    EXPECT_LT(error_frac, 0.01);
  }
}
}  // namespace src