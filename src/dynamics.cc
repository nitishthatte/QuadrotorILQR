#include "src/dynamics.hh"

namespace src {
manif::SE3d dynamics(const manif::SE3d &x, const manif::SE3d &u,
                     manif::SE3d::Jacobian *J_x_ptr,
                     manif::SE3d::Jacobian *J_u_ptr) {
  manif::SE3d::Jacobian J_x, J_u;
  const auto x_next = x.compose(u, J_x, J_u);
  if (J_x_ptr) {
    *J_x_ptr = J_x;
  }
  if (J_u_ptr) {
    *J_u_ptr = J_u;
  }
  return x_next;
}
}  // namespace src