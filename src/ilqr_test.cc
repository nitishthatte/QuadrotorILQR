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

TEST(ILQR, ForwardPassSimulatesTrajectory) {
  const auto N = 3;
  const auto dt_s = 0.1;

  // create cost function
  const CostFunc::CostHessianStateState Q =
      CostFunc::CostHessianStateState::Identity();
  const CostFunc::CostHessianControlControl R =
      CostFunc::CostHessianStateState::Identity();
  auto cost_func = CostFunc(Q, R, {State::Identity()}, {Control::Identity()});

  const auto ilqr = ILQRSolver(cost_func);

  // control policy
  Control::Tangent delta_u = Control::Tangent::Zero();
  delta_u.coeffs()(0) = 1.0;  // delta-x-pos
  const std::vector<Control::Tangent> delta_u_traj{N, delta_u};
  const std::vector<ILQRSolver::FeedbackGains> K{
      N, ILQRSolver::FeedbackGains::Zero()};

  // create current trajectory
  Trajectory<LieDynamics> current_traj{
      N,
      {.time_s = 0.0,
       .state = LieDynamics::State::Identity(),
       .control = LieDynamics::Control::Identity()}};
  auto time_s = 0.0;
  for (auto &pt : current_traj) {
    pt.time_s = time_s;
    time_s += dt_s;
  }

  // create expected new trajectory
  Trajectory<LieDynamics> new_traj_expected{
      {.time_s = 0.0,
       .state = LieDynamics::State::Identity(),
       .control =
           LieDynamics::Control{{1.0, 0.0, 0.0}, manif::SO3d::Identity()}},
      {.time_s = dt_s,
       .state = LieDynamics::State{{1.0, 0.0, 0.0}, manif::SO3d::Identity()},
       .control =
           LieDynamics::Control{{1.0, 0.0, 0.0}, manif::SO3d::Identity()}},
      {.time_s = 2 * dt_s,
       .state = LieDynamics::State{{2.0, 0.0, 0.0}, manif::SO3d::Identity()},
       .control =
           LieDynamics::Control{{1.0, 0.0, 0.0}, manif::SO3d::Identity()}}};

  const auto new_traj = ilqr.forward_pass(current_traj, delta_u_traj, K).first;

  EXPECT_EQ(new_traj, new_traj_expected);
}

}  // namespace src