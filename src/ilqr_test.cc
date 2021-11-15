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

class ILQRForwardPassFixture : public ::testing::Test {
 protected:
  ILQRForwardPassFixture() {
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
    delta_u_traj_ = std::vector<Control::Tangent>{N_, delta_u};
  }

  size_t N_ = 3;
  double dt_s_ = 0.1;
  std::vector<ILQRSolver::FeedbackGains> K_{N_,
                                            ILQRSolver::FeedbackGains::Zero()};
  Trajectory<LieDynamics> current_traj_;
  std::vector<Control::Tangent> delta_u_traj_;

  // create cost function
  CostFunc::CostHessianStateState Q_ =
      CostFunc::CostHessianStateState::Identity();
  CostFunc::CostHessianControlControl R_ =
      CostFunc::CostHessianStateState::Identity();
  CostFunc cost_func_{
      Q_, R_, {N_, State::Identity()}, {N_, Control::Identity()}};
};

TEST_F(ILQRForwardPassFixture, SimulatesTrajectory) {
  const auto ilqr = ILQRSolver(cost_func_);

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

  const auto new_traj =
      ilqr.forward_pass(current_traj_, delta_u_traj_, K_).first;

  EXPECT_EQ(new_traj, new_traj_expected);
}

TEST_F(ILQRForwardPassFixture, CalculatesCorrectCost) {
  const auto ilqr = ILQRSolver(cost_func_);

  const auto cost = ilqr.forward_pass(current_traj_, delta_u_traj_, K_).second;

  const auto expected_cost = 1.0 + 2.0 * 2.0 + 1.0 * 3;

  EXPECT_EQ(cost, expected_cost);
}

TEST_F(ILQRForwardPassFixture, CalculatesDifferentialsIfRequested) {
  const auto ilqr = ILQRSolver(cost_func_);

  std::vector<ILQRSolver::OptDiffs> opt_diffs{N_};
  ilqr.forward_pass(current_traj_, delta_u_traj_, K_, 1.0, &opt_diffs);

  int i = 0;
  for (const auto &diffs : opt_diffs) {
    if (i++ == 0) {
      EXPECT_EQ(diffs.cost_diffs.C_x, CostFunc::CostJacobianState::Zero());
    } else {
      EXPECT_NE(diffs.cost_diffs.C_x, CostFunc::CostJacobianState::Zero());
    }
    EXPECT_NE(diffs.cost_diffs.C_u, CostFunc::CostJacobianControl::Zero());
    EXPECT_NE(diffs.cost_diffs.C_xx, CostFunc::CostHessianStateState::Zero());
    EXPECT_NE(diffs.cost_diffs.C_uu,
              CostFunc::CostHessianControlControl::Zero());
    EXPECT_EQ(diffs.cost_diffs.C_xu, CostFunc::CostHessianStateControl::Zero());

    EXPECT_NE(diffs.dynamics_diffs.J_x, State::Jacobian::Zero());
    EXPECT_NE(diffs.dynamics_diffs.J_u, Control::Jacobian::Zero());
  }
}

}  // namespace src