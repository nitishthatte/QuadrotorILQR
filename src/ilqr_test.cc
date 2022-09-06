#include "src/ilqr.hh"

#include <gtest/gtest.h>
#include <manif/impl/se3/SE3.h>

#include "src/quadrotor_model.hh"
#include "trajectory.hh"

namespace src {

using State = QuadrotorModel::State;
using Control = QuadrotorModel::Control;
using DynamicsDifferentials = QuadrotorModel::DynamicsDifferentials;
using ILQRSolver = ILQR<QuadrotorModel>;
using CostFunc = CostFunction<QuadrotorModel>;

namespace {
State create_identity_state() {
  return State{.inertial_from_body = manif::SE3d::Identity(),
               .body_velocity = manif::SE3Tangentd::Zero()};
}

Trajectory<QuadrotorModel> create_identity_traj(const int num_pts,
                                                const double dt_s) {
  Trajectory<QuadrotorModel> traj;
  traj.reserve(num_pts);
  double time_s = 0.0;
  for (int i = 0; i < num_pts; ++i) {
    traj.emplace_back(
        TrajectoryPoint<QuadrotorModel>{.time_s = time_s,
                                        .state = create_identity_state(),
                                        .control = Control::Zero()});
    time_s += dt_s;
  }
  return traj;
}

bool approx_state_eq(const State &lhs, const State &rhs, const double tol) {
  const auto lhs_body_from_rhs_body =
      lhs.inertial_from_body.inverse() * rhs.inertial_from_body;

  const auto initial_from_body_approx_eq =
      lhs_body_from_rhs_body.log().coeffs().norm() < tol;
  const auto body_vel_approx_eq =
      lhs.body_velocity.coeffs().isApprox(rhs.body_velocity.coeffs()) ||
      (lhs.body_velocity.coeffs() - rhs.body_velocity.coeffs()).isZero(tol);
  return initial_from_body_approx_eq && body_vel_approx_eq;
}

bool approx_control_eq(const Control &lhs, const Control &rhs,
                       const double tol) {
  return lhs.isApprox(rhs, tol) || (lhs - rhs).isZero(tol);
}

void check_approx_traj_eq(const Trajectory<QuadrotorModel> &lhs,
                          const Trajectory<QuadrotorModel> &rhs,
                          const double tol) {
  ASSERT_EQ(lhs.size(), rhs.size());
  for (size_t i = 0; i < lhs.size(); ++i) {
    EXPECT_PRED3(approx_state_eq, lhs[i].state, rhs[i].state, tol);
    EXPECT_PRED3(approx_control_eq, lhs[i].control, rhs[i].control, tol);
  }
}

constexpr auto mass_kg = 1.0;
}  // namespace

class ILQRFixture : public ::testing::Test {
 protected:
  ILQRFixture() {
    ctrl_update_traj_ = ILQRSolver::ControlUpdateTrajectory{
        N_, ILQRSolver::ControlUpdate{
                .ff_update = Control::Ones(),
                .feedback = ILQRSolver::FeedbackGains::Zero()}};
  }

  size_t N_ = 3;
  double dt_s_ = 0.1;
  ILQRSolver::ControlUpdateTrajectory ctrl_update_traj_;

  // create cost function inputs
  CostFunc::CostHessianStateState Q_ =
      CostFunc::CostHessianStateState::Identity();
  CostFunc::CostHessianControlControl R_ =
      CostFunc::CostHessianControlControl::Identity();
  Trajectory<QuadrotorModel> current_traj_ = create_identity_traj(N_, dt_s_);

  ILQRSolver ilqr_{
      QuadrotorModel{
          mass_kg,
          Eigen::Matrix3d::Identity(),  // inertia
          1.0,                          // arm_length_m
          1.0,                          // torque_to_thrust_ratio_m
          0.0                           // g_mpss
      },
      CostFunc{Q_, R_, current_traj_}, dt_s_,
      ILQROptions{.line_search_params = LineSearchParams{0.5, 0.5, 10},
                  .convergence_criteria = {
                      .rtol = 1e-12, .atol = 1e-12, .max_iters = 100}}};
};

TEST_F(ILQRFixture, ForwardSimGeneratesCorrectTrajectory) {
  // expected new trajectory
  const Control u = Control::Ones();
  const auto accel_mpss = u.sum() / mass_kg;

  const State state_0 = create_identity_state();
  State state_1 = state_0;
  state_1.body_velocity.coeffs()(2) = dt_s_ * accel_mpss;

  State state_2 = state_0;
  manif::SE3Tangentd state_2_offset{};
  state_2_offset.coeffs()(2) = dt_s_ * dt_s_ * accel_mpss;
  state_2.inertial_from_body += state_2_offset;
  state_2.body_velocity.coeffs()(2) = 2.0 * dt_s_ * accel_mpss;

  Trajectory<QuadrotorModel> new_traj_expected{
      {.time_s = 0.0, .state = state_0, .control = u},
      {.time_s = dt_s_, .state = state_1, .control = u},
      {.time_s = 2 * dt_s_, .state = state_2, .control = u},
  };

  const auto new_traj = ilqr_.forward_sim(current_traj_, ctrl_update_traj_);

  check_approx_traj_eq(new_traj, new_traj_expected, 1e-6);
}

TEST_F(ILQRFixture, CostTrajectoryCalculatesCorrectCost) {
  const auto new_traj = ilqr_.forward_sim(current_traj_, ctrl_update_traj_);
  const auto cost = ilqr_.cost_trajectory(new_traj);

  const Control u = Control::Ones();
  const auto accel_mpss = u.sum() / mass_kg;
  const auto expected_cost =
      std::pow(dt_s_ * accel_mpss, 2.0) +
      std::pow(dt_s_ * dt_s_ * accel_mpss, 2.0) +
      std::pow(2.0 * dt_s_ * accel_mpss, 2.0)  // state cost
      + 3 * 4;

  EXPECT_DOUBLE_EQ(cost, expected_cost);
}

TEST_F(ILQRFixture, BackwardPassReturnsZeroUpdateIfZeroGradient) {
  const auto [ctrl_traj_update, cost_reduction_terms] =
      ilqr_.backwards_pass(current_traj_);

  EXPECT_EQ(ctrl_traj_update.size(), N_);
  EXPECT_EQ(cost_reduction_terms.QuTk, 0.0);
  EXPECT_EQ(cost_reduction_terms.kTQuuk, 0.0);
  for (const auto &ctrl_update : ctrl_traj_update) {
    EXPECT_EQ(ctrl_update.ff_update, Control::Zero());
  }
}

TEST_F(ILQRFixture,
       BackwardsPassExpectedValueReductionIsNegativeIfReductionPossible) {
  const auto new_traj = ilqr_.forward_sim(current_traj_, ctrl_update_traj_);
  std::vector<ILQRSolver::CostDiffs> cost_diffs{N_};
  ilqr_.cost_trajectory(new_traj);

  const auto cost_reduction_terms = ilqr_.backwards_pass(new_traj).second;

  EXPECT_LT(cost_reduction_terms.QuTk, 0.0);
}

TEST_F(ILQRFixture, LineSearchFindsStepSizeThatReducesCost) {
  const auto traj = ilqr_.forward_sim(current_traj_, ctrl_update_traj_);
  const auto cost = ilqr_.cost_trajectory(traj);
  const auto [ctrl_update_traj, cost_reduction_terms] =
      ilqr_.backwards_pass(traj);
  const auto [new_traj, new_cost, step] =
      ilqr_.line_search(traj, cost, ctrl_update_traj, cost_reduction_terms);

  EXPECT_LT(new_cost - cost,
            ilqr_.options_.line_search_params.desired_reduction_frac *
                detail::calculate_cost_reduction(cost_reduction_terms, step));
}

TEST_F(ILQRFixture, SolveFindsOptimalTrajectory) {
  // Optimal solution should be current traj. Perform forward simulation first
  // so that the initial traj is different.
  for (auto &pt : ctrl_update_traj_) {
    pt.ff_update(0) *= 100;
    pt.ff_update(2) *= 100;
  }
  std::cerr << "current_traj : " << current_traj_ << std::endl;
  const auto initial_traj = ilqr_.forward_sim(current_traj_, ctrl_update_traj_);
  std::cerr << "initial_traj : " << initial_traj << std::endl;
  const auto opt_traj = ilqr_.solve(initial_traj);
  std::cerr << "optimal traj: " << opt_traj << std::endl;

  check_approx_traj_eq(current_traj_, opt_traj, 1e-6);
}
}  // namespace src