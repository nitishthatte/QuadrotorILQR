#include "src/cost.hh"

namespace src {
double CostFunction::operator()(const manif::SE3d &x, const manif::SE3d &u,
                                const int i, CostDifferentials *diffs) const {
  const auto &x_d = desired_states_.at(i);
  const auto &u_d = desired_controls_.at(i);

  const auto delta_x = x.minus(x_d);
  const auto delta_u = u.minus(u_d);

  return (delta_x.coeffs().transpose() * Q_ * delta_x.coeffs() +
          delta_u.coeffs().transpose() * R_ * delta_u.coeffs())(0);
}
}  // namespace src