#include "src/cost.hh"

namespace src {
double CostFunction::operator()(const manif::SE3d &x, const manif::SE3d &u,
                                const int i, CostDifferentials *diffs) const {
  const auto &x_d = desired_states_.at(i);
  const auto &u_d = desired_controls_.at(i);

  manif::SE3d::Tangent::Jacobian J_delta_x, J_delta_u;
  const auto delta_x = x.minus(x_d, J_delta_x);
  const auto delta_u = u.minus(u_d, J_delta_u);

  const auto cost = (delta_x.coeffs().transpose() * Q_ * delta_x.coeffs() +
                     delta_u.coeffs().transpose() * R_ * delta_u.coeffs())(0);

  if (diffs) {
    diffs->C_x = 2 * delta_x.coeffs().transpose() * Q_ * J_delta_x;
    diffs->C_xx = 2 * J_delta_x.transpose() * Q_ * J_delta_x;

    diffs->C_u = 2 * delta_u.coeffs().transpose() * R_ * J_delta_u;
    diffs->C_uu = 2 * J_delta_u.transpose() * R_ * J_delta_u;

    diffs->C_xu = CostHessianStateControl::Zero();
  }

  return cost;
}
}  // namespace src