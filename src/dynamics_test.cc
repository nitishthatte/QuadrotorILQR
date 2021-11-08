#include "src/dynamics.hh"

#include <gtest/gtest.h>

namespace model {
// Demonstrate some basic assertions.
TEST(Dynamics, UpdatesStateCorrectly) {
  const auto x_orig = manif::SE3d::Identity();
  const auto u = manif::SE3d{{1.0, 0.0, 0.0}, manif::SO3d{M_PI, 0.0, 0.0}};

  const auto x_new = dynamics(x_orig, u);
  const auto x_expected = u;

  EXPECT_EQ(x_expected, x_new);
}

TEST(Dynamics, CalculatesCorrectStateJacobian) {
  const auto x_orig = manif::SE3d::Identity();
  const auto u = manif::SE3d{{1.0, 0.0, 0.0}, manif::SO3d{M_PI, 0.0, 0.0}};

  manif::SE3d::Jacobian J_x = manif::SE3d::Jacobian::Zero();
  dynamics(x_orig, u, &J_x);

  const manif::SE3d::Jacobian J_x_expected = u.adj().inverse();
  EXPECT_EQ(J_x, J_x_expected);
}

TEST(Dynamics, CalculatesCorrectControlJacobian) {
  const auto x_orig = manif::SE3d::Identity();
  const auto u = manif::SE3d{{1.0, 0.0, 0.0}, manif::SO3d{M_PI, 0.0, 0.0}};

  manif::SE3d::Jacobian J_u = manif::SE3d::Jacobian::Zero();
  dynamics(x_orig, u, nullptr, &J_u);

  const manif::SE3d::Jacobian J_u_expected = manif::SE3d::Jacobian::Identity();
  EXPECT_EQ(J_u, J_u_expected);
}
}  // namespace model