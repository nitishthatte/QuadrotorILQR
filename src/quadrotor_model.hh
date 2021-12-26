#pragma once
#include <manif/manif.h>

namespace src {
struct QuadrotorModel {
  struct State {
    manif::SE3d inertial_from_body;
    manif::SE3d::Tangent body_velocity;
  };
  struct StateTimeDerivtive {
    manif::SE3d::Tangent body_velocity;
    manif::SE3d::Tangent body_acceleration;
  };
  static constexpr int STATE_DIM = 2 * decltype(State::inertial_from_body)::DoF;
  using StateJacobian = Eigen::Matrix<double, STATE_DIM, STATE_DIM>;

  using Control = Eigen::Vector4d;
  static constexpr int CONTROL_DIM = 4;
  using ControlJacobian = Eigen::Matrix<double, STATE_DIM, CONTROL_DIM>;

  struct DynamicsDifferentials {
    StateJacobian J_x;
    ControlJacobian J_u;
  };

  double mass_kg_;
  Eigen::Matrix3d inertia_;

  State dynamics(const State &x, const Control &u, const double dt_s,
                 DynamicsDifferentials *diffs = nullptr) const;
};

}  // namespace src