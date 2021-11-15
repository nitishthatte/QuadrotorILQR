#include "src/dynamics.hh"

namespace src {
LieDynamics::State LieDynamics::dynamics(const State &x, const Control &u,
                                         DynamicsDifferentials *diffs) {
  State::Jacobian J_x;
  Control::Jacobian J_u;
  const auto x_next = x.compose(u, J_x, J_u);
  if (diffs) {
    diffs->J_x = J_x;
    diffs->J_u = J_u;
  }
  return x_next;
}
}  // namespace src