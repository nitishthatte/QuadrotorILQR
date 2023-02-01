#pragma once
#include <manif/manif.h>

namespace src {

struct QuadrotorModel {
  QuadrotorModel(double mass_kg, const Eigen::Matrix3d &inertia,
                 double arm_length_m, double torque_to_thrust_ratio_m,
                 double g_mpss = 9.81);

  struct State {
    manif::SE3d inertial_from_body;
    manif::SE3Tangentd body_velocity;
  };
  static constexpr int CONFIG_DIM = decltype(State::inertial_from_body)::DoF;
  static constexpr int STATE_DIM = 2 * CONFIG_DIM;

  struct StateTangent {
    manif::SE3Tangentd body_velocity;
    manif::SE3Tangentd body_acceleration;

    Eigen::Vector<double, STATE_DIM> coeffs() const;

    double &operator[](int i);
    const double &operator[](int i) const;

    static StateTangent Zero();
  };
  using StateJacobian = Eigen::Matrix<double, STATE_DIM, STATE_DIM>;
  struct StateBlocks {
    const static inline auto inertial_from_body = Eigen::seqN(0, 6);
    const static inline auto inertial_from_body_pos = Eigen::seqN(0, 3);
    const static inline auto inertial_from_body_rot = Eigen::seqN(3, 3);
    const static inline auto body_velocity = Eigen::seqN(6, 6);
    const static inline auto body_lin_vel = Eigen::seqN(6, 3);
    const static inline auto body_ang_vel = Eigen::seqN(9, 3);
  };

  using Control = Eigen::Vector4d;
  static constexpr int CONTROL_DIM = 4;
  using ControlJacobian = Eigen::Matrix<double, STATE_DIM, CONTROL_DIM>;

  struct DynamicsDifferentials {
    StateJacobian J_x;
    ControlJacobian J_u;
  };

  struct BinaryStateFuncDiffs {
    QuadrotorModel::StateJacobian J_x_lhs;
    QuadrotorModel::StateJacobian J_x_rhs;
  };

  double mass_kg_;
  Eigen::Matrix3d inertia_;
  Eigen::LLT<Eigen::Matrix3d> inertia_llt_;
  double arm_length_m_;
  double torque_to_thrust_ratio_m_;
  Eigen::Matrix<double, 3, 4> moment_arms_;
  double g_mpss_;

  State discrete_dynamics(const State &x, const Control &u, const double dt_s,
                          DynamicsDifferentials *diffs = nullptr) const;

  StateTangent continuous_dynamics(
      const State &x, const Control &u,
      DynamicsDifferentials *diffs = nullptr) const;
};

QuadrotorModel::StateTangent operator*(
    double scalar, const QuadrotorModel::StateTangent &tangent);

QuadrotorModel::StateTangent operator/(
    const QuadrotorModel::StateTangent &tangent, double scalar);

QuadrotorModel::StateTangent operator+(const QuadrotorModel::StateTangent &lhs,
                                       const QuadrotorModel::StateTangent &rhs);

QuadrotorModel::StateTangent operator-(const QuadrotorModel::StateTangent &lhs,
                                       const QuadrotorModel::StateTangent &rhs);

QuadrotorModel::State operator+(const QuadrotorModel::State &x,
                                const QuadrotorModel::StateTangent &tangent);

QuadrotorModel::State add(
    const QuadrotorModel::State &x, const QuadrotorModel::StateTangent &tangent,
    QuadrotorModel::BinaryStateFuncDiffs *diffs = nullptr);

QuadrotorModel::State operator-(const QuadrotorModel::State &x,
                                const QuadrotorModel::StateTangent &tangent);

QuadrotorModel::StateTangent operator-(const QuadrotorModel::State &lhs,
                                       const QuadrotorModel::State &rhs);

QuadrotorModel::StateTangent minus(const QuadrotorModel::State &lhs,
                                   const QuadrotorModel::State &rhs,
                                   QuadrotorModel::BinaryStateFuncDiffs *diffs);

bool operator==(const QuadrotorModel::State &lhs,
                const QuadrotorModel::State &rhs);

std::ostream &operator<<(std::ostream &out, const QuadrotorModel::State &state);

namespace detail {
QuadrotorModel::State euler_step(
    const QuadrotorModel::State &x, const QuadrotorModel::StateTangent &x_dot,
    double dt_s, QuadrotorModel::BinaryStateFuncDiffs *diffs = nullptr);
}
}  // namespace src