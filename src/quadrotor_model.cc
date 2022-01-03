#include "src/quadrotor_model.hh"

namespace src {
namespace {
constexpr auto g = 9.81;
}  // namespace

QuadrotorModel::QuadrotorModel(const double mass_kg,
                               const Eigen::Matrix3d &inertia,
                               const double arm_length_m,
                               const double torque_to_thrust_ratio_m)
    : mass_kg_{mass_kg},
      inertia_{inertia},
      arm_length_m_{arm_length_m},
      torque_to_thrust_ratio_m_{torque_to_thrust_ratio_m} {
  inertia_llt_ = inertia_.llt();
  if (inertia_llt_.info() == Eigen::NumericalIssue ||
      !inertia_.isApprox(inertia_.transpose())) {
    throw std::runtime_error("Inertia matrix is not positive definite!");
  }
}

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

    // fill in dv/dv
    diffs->J_x(StateBlocks::inertial_from_body, StateBlocks::body_velocity) =
        Eigen::Matrix<double, CONFIG_DIM, CONFIG_DIM>::Identity();

    // fill in dv_lin_dot/dR
    const Eigen::Vector3d RTez =
        x.inertial_from_body.rotation().transpose() * Eigen::Vector3d::UnitZ();
    const Eigen::Matrix3d RTez_hat{
        {0, -RTez.z(), RTez.y()},
        {RTez.z(), 0, -RTez.x()},
        {-RTez.y(), RTez.x(), 0},
    };
    diffs->J_x(StateBlocks::body_lin_vel, StateBlocks::inertial_from_body_rot) =
        -g * RTez_hat;

    // fill in dv_ang_dot/dv_ang
    const Eigen::Vector<double, 3> &omega = x.body_velocity.ang();
    const Eigen::Matrix3d omega_hat{{0, -omega.z(), omega.y()},
                                    {omega.z(), 0, -omega.x()},
                                    {-omega.y(), omega.x(), 0}};

    const Eigen::Vector<double, 3> Jomega = inertia_ * omega;
    const Eigen::Matrix3d Jomega_hat{{0, -Jomega.z(), Jomega.y()},
                                     {Jomega.z(), 0, -Jomega.x()},
                                     {-Jomega.y(), Jomega.x(), 0}};
    const Eigen::Matrix3d Jomega_diff = omega_hat * inertia_ - Jomega_hat;

    diffs->J_x(StateBlocks::body_ang_vel, StateBlocks::body_ang_vel) =
        -inertia_llt_.solve(Jomega_diff);
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

QuadrotorModel::State add(const QuadrotorModel::State &x,
                          const QuadrotorModel::StateTangent &tangent,
                          QuadrotorModel::StateJacobian *J_x_ptr,
                          QuadrotorModel::StateJacobian *J_t_ptr) {
  constexpr auto CONFIG_DIM = QuadrotorModel::CONFIG_DIM;
  using StateBlocks = QuadrotorModel::StateBlocks;
  if (J_x_ptr && J_t_ptr) {
    Eigen::Matrix<double, CONFIG_DIM, CONFIG_DIM> J_plus_x_inertial_from_body;
    Eigen::Matrix<double, CONFIG_DIM, CONFIG_DIM> J_plus_tangent_body_vel;
    const QuadrotorModel::State x_next{
        .inertial_from_body = x.inertial_from_body.plus(
            tangent.body_velocity, J_plus_x_inertial_from_body,
            J_plus_tangent_body_vel),
        .body_velocity = x.body_velocity + tangent.body_acceleration};

    auto &J_x = *J_x_ptr;
    J_x = QuadrotorModel::StateJacobian::Identity();
    J_x(StateBlocks::inertial_from_body, StateBlocks::inertial_from_body) =
        J_plus_x_inertial_from_body;

    auto &J_t = *J_t_ptr;
    J_t = QuadrotorModel::StateJacobian::Identity();
    J_t(StateBlocks::inertial_from_body, StateBlocks::inertial_from_body) =
        J_plus_tangent_body_vel;

    return x_next;
  }
  return x + tangent;
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

QuadrotorModel::StateTangent operator-(const QuadrotorModel::State &lhs,
                                       const QuadrotorModel::State &rhs) {
  return {.body_velocity = lhs.inertial_from_body - rhs.inertial_from_body,
          .body_acceleration = lhs.body_velocity - rhs.body_velocity};
}

QuadrotorModel::State euler_step(
    const QuadrotorModel::State &x, const QuadrotorModel::StateTangent &x_dot,
    const double dt_s,
    QuadrotorModel::DynamicsDifferentials *cont_dynamics_diffs,
    QuadrotorModel::DynamicsDifferentials *diffs_out) {
  return x + dt_s * x_dot;
}
}  // namespace src