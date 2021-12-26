#include "src/quadrotor_model.hh"

namespace src {
namespace {
constexpr auto g = 9.81;
}
QuadrotorModel::State QuadrotorModel::dynamics(
    const State &x, const Control &u, const double dt_s,
    DynamicsDifferentials *diffs) const {
  // StateJacobian J_x;
  // ControlJacobian J_u;
  // const auto x_next = x.origin_from_com.compose(u, J_x, J_u);
  QuadrotorModel::StateTimeDerivtive xdot;
  xdot.body_velocity = x.body_velocity;
  xdot.body_acceleration.lin() =
      (-g * x.inertial_from_body.rotation().transpose() *
           Eigen::Vector3d::UnitZ() +
       u.sum() * Eigen::Vector3d::UnitZ()) /
      mass_kg_;
  xdot.body_acceleration.ang() = Eigen::Vector3d::Zero();

  auto x_next = x;
  x_next.inertial_from_body =
      x_next.inertial_from_body + xdot.body_velocity * dt_s;
  x_next.body_velocity = x_next.body_velocity + xdot.body_acceleration * dt_s;

  /*
  if (diffs) {
    diffs->J_x = StateJacobian::Zero();
    diffs->J_u = ControlJacobian::Zero();
  }
  */
  return x_next;
}
}  // namespace src