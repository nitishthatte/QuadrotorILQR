#include "src/quadrotor_model.hh"

namespace src {
namespace {
constexpr auto g = 9.81;

QuadrotorModel::State euler_step(
    const QuadrotorModel::State &x,
    const QuadrotorModel::StateTimeDerivative &xdot, const double dt_s) {
  auto x_next = x;
  x_next.inertial_from_body =
      x_next.inertial_from_body + xdot.body_velocity * dt_s;
  x_next.body_velocity = x_next.body_velocity + xdot.body_acceleration * dt_s;
  return x_next;
}

QuadrotorModel::StateTimeDerivative operator*(
    const double scalar, const QuadrotorModel::StateTimeDerivative &dx_dt) {
  return {.body_velocity = scalar * dx_dt.body_velocity,
          .body_acceleration = scalar * dx_dt.body_acceleration};
}

QuadrotorModel::StateTimeDerivative operator/(
    const QuadrotorModel::StateTimeDerivative &dx_dt, const double scalar) {
  return {.body_velocity = dx_dt.body_velocity / scalar,
          .body_acceleration = dx_dt.body_acceleration / scalar};
}

QuadrotorModel::StateTimeDerivative operator+(
    const QuadrotorModel::StateTimeDerivative &lhs,
    const QuadrotorModel::StateTimeDerivative &rhs) {
  return {.body_velocity = lhs.body_velocity + rhs.body_velocity,
          .body_acceleration = lhs.body_acceleration + rhs.body_acceleration};
}

}  // namespace

// uses rk4 to integrate the discrete dynamics
QuadrotorModel::State QuadrotorModel::discrete_dynamics(
    const State &x, const Control &u, const double dt_s,
    DynamicsDifferentials *diffs) const {
  // StateJacobian J_x;
  // ControlJacobian J_u;
  // const auto x_next = x.origin_from_com.compose(u, J_x, J_u);
  const auto k1 = continuous_dynamics(x, u);
  const auto k2 = continuous_dynamics(euler_step(x, k1, dt_s / 2), u);
  const auto k3 = continuous_dynamics(euler_step(x, k2, dt_s / 2), u);
  const auto k4 = continuous_dynamics(euler_step(x, k3, dt_s), u);

  const auto xdot = (k1 + 2 * k2 + 2 * k3 + k4) / 6.0;
  return euler_step(x, xdot, dt_s);

  /*
  if (diffs) {
    diffs->J_x = StateJacobian::Zero();
    diffs->J_u = ControlJacobian::Zero();
  }
  */
}

QuadrotorModel::StateTimeDerivative QuadrotorModel::continuous_dynamics(
    const State &x, const Control &u, DynamicsDifferentials *diffs) const {
  QuadrotorModel::StateTimeDerivative xdot;
  xdot.body_velocity = x.body_velocity;
  xdot.body_acceleration.lin() =
      (-g * x.inertial_from_body.rotation().transpose() *
           Eigen::Vector3d::UnitZ() +
       u.sum() * Eigen::Vector3d::UnitZ()) /
      mass_kg_;

  xdot.body_acceleration.ang() = Eigen::Vector3d::Zero();

  return xdot;
}
}  // namespace src