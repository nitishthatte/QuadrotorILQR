#include "src/dynamics.hh"

namespace src {
manif::SE3d dynamics(const manif::SE3d &x, const manif::SE3d &u,
                     manif::SE3d::Jacobian *J_x, manif::SE3d::Jacobian *J_u) {
  manif::SE3d::Jacobian J_x_internal, J_u_internal;
  const auto x_next = x.compose(u, J_x_internal, J_u_internal);
  if (J_x) {
    *J_x = J_x_internal;
  }
  if (J_u) {
    *J_u = J_u_internal;
  }
  return x_next;
}
}  // namespace src