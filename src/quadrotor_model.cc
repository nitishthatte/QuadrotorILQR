#include "src/quadrotor_model.hh"

namespace src {
namespace {
constexpr auto g = 9.81;

QuadrotorModel::State euler_step(const QuadrotorModel::State &x,
                                 const QuadrotorModel::StateTangent &x_dot,
                                 const double dt_s) {
  return x + dt_s * x_dot;
}

}  // namespace

// uses rk4 to integrate the continuous dynamics
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

QuadrotorModel::StateTangent QuadrotorModel::continuous_dynamics(
    const State &x, const Control &u, DynamicsDifferentials *diffs) const {
  StateTangent xdot;
  xdot.body_velocity = x.body_velocity;
  xdot.body_acceleration.lin() =
      (-g * x.inertial_from_body.rotation().transpose() *
           Eigen::Vector3d::UnitZ() +
       u.sum() * Eigen::Vector3d::UnitZ()) /
      mass_kg_;

  const Eigen::Vector3d M_Nm =
      Eigen::Matrix<double, 3, 4>{
          {0, -arm_length_m_, 0, arm_length_m_},
          {arm_length_m_, 0.0, -arm_length_m_, 0.0},
          {-torque_to_thrust_ratio_m_, torque_to_thrust_ratio_m_,
           -torque_to_thrust_ratio_m_, torque_to_thrust_ratio_m_}} *
      u;

  xdot.body_acceleration.ang() =
      inertia_.inverse() * (M_Nm - (xdot.body_velocity.asSO3().hat() *
                                    inertia_ * xdot.body_velocity.ang()));

  if (diffs) {
    diffs->J_x = StateJacobian::Zero();

    // fill in dv_dot/dR
    const Eigen::Vector3d RTez =
        x.inertial_from_body.rotation().transpose() * Eigen::Vector3d::UnitZ();
    const Eigen::Matrix3d RTez_hat = Eigen::Matrix3d{
        {0, -RTez.z(), RTez.y()},
        {RTez.z(), 0, -RTez.x()},
        {-RTez.y(), RTez.x(), 0},
    };
    diffs->J_x(StateBlocks::body_lin_vel, StateBlocks::inertial_from_body_rot) =
        -g * RTez_hat;
  }

  return xdot;
}

Eigen::Vector<double, QuadrotorModel::STATE_DIM>
QuadrotorModel::StateTangent::coeffs() {
  Eigen::Vector<double, QuadrotorModel::STATE_DIM> coeffs;
  coeffs(Eigen::seq(0, QuadrotorModel::STATE_DIM / 2 - 1)) =
      this->body_velocity.coeffs();
  coeffs(Eigen::seq(QuadrotorModel::STATE_DIM / 2, Eigen::last)) =
      this->body_acceleration.coeffs();
  return coeffs;
}

QuadrotorModel::StateTangent operator*(
    const double scalar, const QuadrotorModel::StateTangent &tangent) {
  return {.body_velocity = scalar * tangent.body_velocity,
          .body_acceleration = scalar * tangent.body_acceleration};
}

QuadrotorModel::StateTangent operator/(
    const QuadrotorModel::StateTangent &tangent, const double scalar) {
  return {.body_velocity = tangent.body_velocity / scalar,
          .body_acceleration = tangent.body_acceleration / scalar};
}

QuadrotorModel::StateTangent operator+(
    const QuadrotorModel::StateTangent &lhs,
    const QuadrotorModel::StateTangent &rhs) {
  return {.body_velocity = lhs.body_velocity + rhs.body_velocity,
          .body_acceleration = lhs.body_acceleration + rhs.body_acceleration};
}

QuadrotorModel::StateTangent operator-(
    const QuadrotorModel::StateTangent &lhs,
    const QuadrotorModel::StateTangent &rhs) {
  return {.body_velocity = lhs.body_velocity - rhs.body_velocity,
          .body_acceleration = lhs.body_acceleration - rhs.body_acceleration};
}

QuadrotorModel::State operator+(const QuadrotorModel::State &x,
                                const QuadrotorModel::StateTangent &tangent) {
  return {.inertial_from_body = x.inertial_from_body + tangent.body_velocity,
          .body_velocity = x.body_velocity + tangent.body_acceleration};
}

QuadrotorModel::State operator-(const QuadrotorModel::State &x,
                                const QuadrotorModel::StateTangent &tangent) {
  return {
      .inertial_from_body = x.inertial_from_body + -1 * tangent.body_velocity,
      .body_velocity = x.body_velocity - tangent.body_acceleration};
}
}  // namespace src