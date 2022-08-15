#include "src/quadrotor_model.hh"

#include <array>
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
      torque_to_thrust_ratio_m_{torque_to_thrust_ratio_m},
      moment_arms_{{0, -arm_length_m_, 0, arm_length_m_},
                   {arm_length_m_, 0.0, -arm_length_m_, 0.0},
                   {-torque_to_thrust_ratio_m_, torque_to_thrust_ratio_m_,
                    -torque_to_thrust_ratio_m_, torque_to_thrust_ratio_m_}} {
  inertia_llt_ = inertia_.llt();
  if (inertia_llt_.info() == Eigen::NumericalIssue ||
      !inertia_.isApprox(inertia_.transpose())) {
    throw std::runtime_error("Inertia matrix is not positive definite!");
  }
}

QuadrotorModel::StateTangent QuadrotorModel::StateTangent::Zero() {
  return {.body_velocity = manif::SE3Tangentd::Zero(),
          .body_acceleration = manif::SE3Tangentd::Zero()};
}

// uses euler integration to integrate the continuous dynamics
QuadrotorModel::State QuadrotorModel::discrete_dynamics(
    const State &x, const Control &u, const double dt_s,
    DynamicsDifferentials *diffs) const {
  const auto x_dot = continuous_dynamics(x, u, diffs);

  QuadrotorModel::BinaryStateFuncDiffs *euler_diffs_ptr =
      diffs ? new QuadrotorModel::BinaryStateFuncDiffs{} : nullptr;
  const auto x_next = detail::euler_step(x, x_dot, dt_s, euler_diffs_ptr);

  if (euler_diffs_ptr && diffs) {
    diffs->J_x =
        euler_diffs_ptr->J_x_lhs + euler_diffs_ptr->J_x_rhs * diffs->J_x;
    diffs->J_u = euler_diffs_ptr->J_x_rhs * diffs->J_u;
  }

  return x_next;
}

/*
  constexpr std::array<float, 4> coeffs{1.0 / 6.0, 2.0 / 6.0, 2.0 / 6.0,
                                        1.0 / 6.0};
  const double half_dt_s = dt_s / 2.0;
  const std::array<double, 4> dt_s_table{0.0, half_dt_s, half_dt_s, dt_s};

  auto k = QuadrotorModel::StateTangent::Zero();
  auto x_dot = QuadrotorModel::StateTangent::Zero();
  for (int i = 0; i < 4; ++i) {
    k = continuous_dynamics(detail::euler_step(x, k, dt_s_table[i]), u);
    x_dot = x_dot + coeffs[i] * k;
  }
*/

QuadrotorModel::StateTangent QuadrotorModel::continuous_dynamics(
    const State &x, const Control &u, DynamicsDifferentials *diffs) const {
  StateTangent xdot;
  xdot.body_velocity = x.body_velocity;
  xdot.body_acceleration.lin() =
      (-g * x.inertial_from_body.rotation().transpose() *
           Eigen::Vector3d::UnitZ() +
       u.sum() * Eigen::Vector3d::UnitZ()) /
      mass_kg_;

  const Eigen::Vector3d M_Nm = moment_arms_ * u;

  xdot.body_acceleration.ang() =
      inertia_llt_.solve((M_Nm - (xdot.body_velocity.asSO3().hat() * inertia_ *
                                  xdot.body_velocity.ang())));

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

    diffs->J_u = ControlJacobian::Zero();

    diffs->J_u(StateBlocks::body_lin_vel[2], Eigen::all) =
        Eigen::Matrix<double, 1, 4>::Constant(1.0 / mass_kg_);

    diffs->J_u(StateBlocks::body_ang_vel, Eigen::all) =
        inertia_llt_.solve(moment_arms_);
  }
  return xdot;
}

Eigen::Vector<double, QuadrotorModel::STATE_DIM>
QuadrotorModel::StateTangent::coeffs() const {
  Eigen::Vector<double, QuadrotorModel::STATE_DIM> coeffs;
  coeffs(Eigen::seq(0, QuadrotorModel::STATE_DIM / 2 - 1)) =
      this->body_velocity.coeffs();
  coeffs(Eigen::seq(QuadrotorModel::STATE_DIM / 2, Eigen::last)) =
      this->body_acceleration.coeffs();
  return coeffs;
}

double &QuadrotorModel::StateTangent::operator[](int i) {
  if (i < CONFIG_DIM) {
    return this->body_velocity.coeffs()[i];
  }
  return this->body_acceleration.coeffs()[i - CONFIG_DIM];
}

const double &QuadrotorModel::StateTangent::operator[](int i) const {
  if (i < CONFIG_DIM) {
    return this->body_velocity.coeffs()[i];
  }
  return this->body_acceleration.coeffs()[i - CONFIG_DIM];
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
                          QuadrotorModel::BinaryStateFuncDiffs *diffs) {
  constexpr auto CONFIG_DIM = QuadrotorModel::CONFIG_DIM;
  using StateBlocks = QuadrotorModel::StateBlocks;
  if (diffs) {
    Eigen::Matrix<double, CONFIG_DIM, CONFIG_DIM> J_plus_x_inertial_from_body;
    Eigen::Matrix<double, CONFIG_DIM, CONFIG_DIM> J_plus_tangent_body_vel;
    const QuadrotorModel::State added{
        .inertial_from_body = x.inertial_from_body.plus(
            tangent.body_velocity, J_plus_x_inertial_from_body,
            J_plus_tangent_body_vel),
        .body_velocity = x.body_velocity + tangent.body_acceleration};

    diffs->J_x_lhs = QuadrotorModel::StateJacobian::Identity();  // df/dx
    diffs->J_x_lhs(StateBlocks::inertial_from_body,
                   StateBlocks::inertial_from_body) =
        J_plus_x_inertial_from_body;

    diffs->J_x_rhs = QuadrotorModel::StateJacobian::Identity();  // df/dt
    diffs->J_x_rhs(StateBlocks::inertial_from_body,
                   StateBlocks::inertial_from_body) = J_plus_tangent_body_vel;

    return added;
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

QuadrotorModel::StateTangent minus(
    const QuadrotorModel::State &lhs, const QuadrotorModel::State &rhs,
    QuadrotorModel::BinaryStateFuncDiffs *diffs) {
  constexpr auto CONFIG_DIM = QuadrotorModel::CONFIG_DIM;
  using StateBlocks = QuadrotorModel::StateBlocks;
  if (diffs) {
    Eigen::Matrix<double, CONFIG_DIM, CONFIG_DIM>
        J_minus_lhs_inertial_from_body;
    Eigen::Matrix<double, CONFIG_DIM, CONFIG_DIM>
        J_minus_rhs_inertial_from_body;
    const QuadrotorModel::StateTangent difference{
        .body_velocity = lhs.inertial_from_body.minus(
            rhs.inertial_from_body, J_minus_lhs_inertial_from_body,
            J_minus_rhs_inertial_from_body),
        .body_acceleration = lhs.body_velocity - rhs.body_velocity};

    diffs->J_x_lhs = QuadrotorModel::StateJacobian::Identity();  // df/dlhs
    diffs->J_x_lhs(StateBlocks::inertial_from_body,
                   StateBlocks::inertial_from_body) =
        J_minus_lhs_inertial_from_body;

    diffs->J_x_rhs = -QuadrotorModel::StateJacobian::Identity();  // df/drhs
    diffs->J_x_rhs(StateBlocks::inertial_from_body,
                   StateBlocks::inertial_from_body) =
        J_minus_rhs_inertial_from_body;

    return difference;
  }
  return lhs - rhs;
}

bool operator==(const QuadrotorModel::State &lhs,
                const QuadrotorModel::State &rhs) {
  return lhs.body_velocity == rhs.body_velocity &&
         lhs.inertial_from_body == rhs.inertial_from_body;
}

std::ostream &operator<<(std::ostream &out,
                         const QuadrotorModel::State &state) {
  out << "intertial_from_body: " << state.inertial_from_body
      << ", body velocity: " << state.body_velocity;
  return out;
}

namespace detail {
QuadrotorModel::State euler_step(const QuadrotorModel::State &x,
                                 const QuadrotorModel::StateTangent &x_dot,
                                 const double dt_s,
                                 QuadrotorModel::BinaryStateFuncDiffs *diffs) {
  if (diffs) {
    const auto x_next = add(x, dt_s * x_dot, diffs);
    diffs->J_x_rhs *= dt_s;  // scale df/dxdot
    return x_next;
  }
  return x + dt_s * x_dot;
}
}  // namespace detail
}  // namespace src