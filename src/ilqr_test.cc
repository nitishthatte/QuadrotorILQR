#include "src/ilqr.hh"

#include <gtest/gtest.h>
#include <manif/impl/se3/SE3.h>

#include "dynamics.hh"
#include "trajectory.hh"

namespace src {

using State = LieDynamics::State;
using Control = LieDynamics::Control;
using DynamicsDifferentials = LieDynamics::DynamicsDifferentials;
using ILQRSolver = ILQR<LieDynamics>;
using CostFunc = CostFunction<LieDynamics>;

class ILQRFixture : public ::testing::Test {
 protected:
  ILQRFixture() {
    current_traj_ =
        Trajectory<LieDynamics>{N_,
                                {.time_s = 0.0,
                                 .state = LieDynamics::State::Identity(),
                                 .control = LieDynamics::Control::Identity()}};
    auto time_s = 0.0;
    for (auto &pt : current_traj_) {
      pt.time_s = time_s;
      time_s += dt_s_;
    }

    Control::Tangent delta_u = Control::Tangent::Zero();
    delta_u.coeffs()(0) = 1.0;  // delta-x-pos
    ctrl_update_traj_ = ILQRSolver::ControlUpdateTrajectory{
        N_, ILQRSolver::ControlUpdate{
                .ff_update = delta_u,
                .feedback = ILQRSolver::FeedbackGains::Zero()}};
  }

  size_t N_ = 3;
  double dt_s_ = 0.1;
  Trajectory<LieDynamics> current_traj_;
  ILQRSolver::ControlUpdateTrajectory ctrl_update_traj_;

  // create cost function
  CostFunc::CostHessianStateState Q_ =
      CostFunc::CostHessianStateState::Identity();
  CostFunc::CostHessianControlControl R_ =
      CostFunc::CostHessianStateState::Identity();

  ILQRSolver ilqr_{
      CostFunc{Q_, R_, {N_, State::Identity()}, {N_, Control::Identity()}},
      LineSearchParams{0.5, 0.5, 10}};
};

TEST_F(ILQRFixture, ForwardPassSimulatesTrajectory) {
  // expected new trajectory
  Trajectory<LieDynamics> new_traj_expected{
      {.time_s = 0.0,
       .state = LieDynamics::State::Identity(),
       .control =
           LieDynamics::Control{{1.0, 0.0, 0.0}, manif::SO3d::Identity()}},
      {.time_s = dt_s_,
       .state = LieDynamics::State{{1.0, 0.0, 0.0}, manif::SO3d::Identity()},
       .control =
           LieDynamics::Control{{1.0, 0.0, 0.0}, manif::SO3d::Identity()}},
      {.time_s = 2 * dt_s_,
       .state = LieDynamics::State{{2.0, 0.0, 0.0}, manif::SO3d::Identity()},
       .control =
           LieDynamics::Control{{1.0, 0.0, 0.0}, manif::SO3d::Identity()}}};

  const auto new_traj = ilqr_.forward_pass(current_traj_, ctrl_update_traj_);

  EXPECT_EQ(new_traj, new_traj_expected);
}

TEST_F(ILQRFixture, ForwardPassCalculatesDifferentialsIfRequested) {
  std::vector<ILQRSolver::DynamicsDiffs> fwd_diffs{N_};
  ilqr_.forward_pass(current_traj_, ctrl_update_traj_, 1.0, &fwd_diffs);

  // int i = 0;
  for (const auto &diffs : fwd_diffs) {
    // if (i++ == 0) {
    //   EXPECT_EQ(diffs.cost_diffs.x, CostFunc::CostJacobianState::Zero());
    // } else {
    //   EXPECT_NE(diffs.cost_diffs.x, CostFunc::CostJacobianState::Zero());
    // }
    // EXPECT_NE(diffs.cost_diffs.u, CostFunc::CostJacobianControl::Zero());
    // EXPECT_NE(diffs.cost_diffs.xx, CostFunc::CostHessianStateState::Zero());
    // EXPECT_NE(diffs.cost_diffs.uu,
    // CostFunc::CostHessianControlControl::Zero());
    // EXPECT_EQ(diffs.cost_diffs.xu,
    // CostFunc::CostHessianStateControl::Zero());

    EXPECT_NE(diffs.J_x, State::Jacobian::Zero());
    EXPECT_NE(diffs.J_u, Control::Jacobian::Zero());
  }
}

TEST_F(ILQRFixture, CostTrajectoryCalculatesCorrectCost) {
  const auto new_traj = ilqr_.forward_pass(current_traj_, ctrl_update_traj_);
  const auto cost = ilqr_.cost_trajectory(new_traj);

  const auto expected_cost = 1.0 + 2.0 * 2.0 + 1.0 * 3;

  EXPECT_EQ(cost, expected_cost);
}

TEST_F(ILQRFixture, BackwardPassReturnsZeroUpdateIfZeroGradient) {
  constexpr size_t num_pts = 4;
  std::vector<ILQRSolver::DynamicsDiffs> dynamics_diffs{
      num_pts,
      {.J_x = State::Jacobian::Zero(), .J_u = Control::Jacobian::Zero()}};
  std::vector<ILQRSolver::CostDiffs> cost_diffs{
      num_pts,
      {
          .x = CostFunc::CostJacobianState::Zero(),
          .u = CostFunc::CostJacobianControl::Zero(),
          .xx = CostFunc::CostHessianStateState::Identity(),
          .uu = CostFunc::CostHessianControlControl::Identity(),
          .xu = CostFunc::CostHessianStateControl::Zero(),
      }};

  const auto [ctrl_traj_update, expected_cost_reduction] =
      ilqr_.backwards_pass(dynamics_diffs, cost_diffs);

  EXPECT_EQ(ctrl_traj_update.size(), num_pts);
  EXPECT_EQ(expected_cost_reduction, 0.0);
  for (const auto &ctrl_update : ctrl_traj_update) {
    EXPECT_EQ(ctrl_update.ff_update, Control::Tangent::Zero());
    EXPECT_EQ(ctrl_update.feedback, ILQRSolver::FeedbackGains::Zero());
  }
}

TEST_F(ILQRFixture,
       BackwardsPassExpectedValueReductionIsNegativeIfReductionPossible) {
  std::vector<ILQRSolver::DynamicsDiffs> dynamics_diffs{N_};
  const auto new_traj = ilqr_.forward_pass(current_traj_, ctrl_update_traj_,
                                           1.0, &dynamics_diffs);
  std::vector<ILQRSolver::CostDiffs> cost_diffs{N_};
  ilqr_.cost_trajectory(new_traj, &cost_diffs);

  const auto expected_cost_reduction =
      ilqr_.backwards_pass(dynamics_diffs, cost_diffs).second;

  EXPECT_LT(expected_cost_reduction, 0.0);
}

TEST_F(ILQRFixture, LineSearchFindsStepSizeThatReducesCost) {
  std::vector<ILQRSolver::DynamicsDiffs> dynamics_diffs{N_};
  const auto traj = ilqr_.forward_pass(current_traj_, ctrl_update_traj_, 1.0,
                                       &dynamics_diffs);
  std::vector<ILQRSolver::CostDiffs> cost_diffs{N_};
  const auto cost = ilqr_.cost_trajectory(traj, &cost_diffs);
  const auto [ctrl_update_traj, expected_cost_reduction] =
      ilqr_.backwards_pass(dynamics_diffs, cost_diffs);
  const auto [new_traj, new_cost, step] =
      ilqr_.line_search(traj, cost, ctrl_update_traj, expected_cost_reduction);

  EXPECT_LT(new_cost - cost,
            expected_cost_reduction *
                ilqr_.line_search_params_.desired_reduction_frac * step);
}
}  // namespace src