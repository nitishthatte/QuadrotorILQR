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
                                        .control = Control::Identity()});
    time_s += dt_s;
  }
  return traj;
}

bool approx_eq(const State &lhs, const State &rhs, const double tol) {
  const auto lhs_body_from_rhs_body =
      lhs.inertial_from_body.inverse() * rhs.inertial_from_body;
  return lhs_body_from_rhs_body.coeffs().isApproxToConstant(0.0, tol) &&
         lhs.body_velocity.coeffs().isApprox(rhs.body_velocity.coeffs());
}

constexpr auto mass_kg = 1.0;
}  // namespace

class ILQRFixture : public ::testing::Test {
 protected:
  ILQRFixture() {
    Control delta_u = Control::Zero();
    delta_u(0) = 1.0;
    ctrl_update_traj_ = ILQRSolver::ControlUpdateTrajectory{
        N_, ILQRSolver::ControlUpdate{
                .ff_update = delta_u,
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
          1.0                           // torque_to_thrust_ratio_m
      },
      CostFunc{Q_, R_, current_traj_}, dt_s_,
      ILQROptions{.line_search_params = LineSearchParams{0.5, 0.5, 10},
                  .convergence_criteria = {
                      .rtol = 1e-12, .atol = 1e-12, .max_iters = 100}}};
};

TEST_F(ILQRFixture, ForwardSimGeneratesCorrectTrajectory) {
  // expected new trajectory
  const State state_0 = create_identity_state();
  State state_1 = state_0;
  state_1.body_velocity.coeffs()(0) = dt_s_;
  State state_2 = state_0;
  state_2.body_velocity.coeffs()(0) = 2.0 * dt_s_;
  state_2.inertial_from_body.translation()(0) = dt_s_ * dt_s_;
  const Control control{1.0, 0.0, 0.0, 0.0};

  Trajectory<QuadrotorModel> new_traj_expected{
      {.time_s = 0.0, .state = state_0, .control = control},
      {.time_s = dt_s_, .state = state_1, .control = control},
      {.time_s = 2 * dt_s_, .state = state_2, .control = control},
  };

  const auto new_traj = ilqr_.forward_sim(current_traj_, ctrl_update_traj_);

  EXPECT_EQ(new_traj, new_traj_expected);
}

TEST_F(ILQRFixture, CostTrajectoryCalculatesCorrectCost) {
  const auto new_traj = ilqr_.forward_sim(current_traj_, ctrl_update_traj_);
  const auto cost = ilqr_.cost_trajectory(new_traj);

  const auto expected_cost = dt_s_ * dt_s_ + 5.0 * dt_s_ * dt_s_ + 1.0 * 3;

  EXPECT_EQ(cost, expected_cost);
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
  const auto initial_traj = ilqr_.forward_sim(current_traj_, ctrl_update_traj_);
  const auto opt_traj = ilqr_.solve(initial_traj);

  ASSERT_EQ(opt_traj.size(), current_traj_.size());
  for (size_t i = 0; i < opt_traj.size(); ++i) {
    EXPECT_PRED3(approx_eq, current_traj_[i].state, opt_traj[i].state, 1e-6);
    EXPECT_TRUE(current_traj_[i].control.isApprox(opt_traj[i].control, 1e-6));
  }
}
}  // namespace src