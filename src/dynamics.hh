#pragma once
#include <manif/manif.h>

namespace src {
struct LieDynamics {
  using State = manif::SE3d;
  using Control = manif::SE3d;
  static constexpr int STATE_DIM = State::DoF;
  static constexpr int CONTROL_DIM = Control::DoF;

  struct DynamicsDifferentials {
    State::Jacobian J_x;
    Control::Jacobian J_u;
  };

  static State dynamics(const State &x, const Control &u,
                        DynamicsDifferentials *diffs = nullptr);
};
}  // namespace src