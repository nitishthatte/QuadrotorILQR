#include "src/cost.hh"

#include <gtest/gtest.h>
#include <manif/impl/se3/SE3.h>

#include "src/quadrotor_model.hh"
#include "trajectory.hh"

namespace src {

using State = QuadrotorModel::State;
using StateTangent = QuadrotorModel::StateTangent;
constexpr auto STATE_DIM = QuadrotorModel::STATE_DIM;
using Control = QuadrotorModel::Control;
constexpr auto CONTROL_DIM = QuadrotorModel::CONTROL_DIM;
using CostFunc = CostFunction<QuadrotorModel>;

namespace {
Trajectory<QuadrotorModel> create_random_traj() {
  return {{.time_s = 0.0,
           .state = State{.inertial_from_body = manif::SE3d::Random(),
                          .body_velocity = manif::SE3Tangentd::Random()},
           .control = Control::Random()}};
}
}  // namespace

TEST(ComputeCost, ReturnsZeroCostWhenZeroError) {
  const CostFunc::CostHessianStateState Q =
      CostFunc::CostHessianStateState::Identity();
  const CostFunc::CostHessianControlControl R =
      CostFunc::CostHessianControlControl::Identity();
  const auto traj = create_random_traj();

  const auto compute_cost = CostFunc(Q, R, traj);

  const auto cost = compute_cost(traj.front().state, traj.front().control, 0);

  EXPECT_EQ(cost, 0.0);
}

class ComputeCostDifferentials : public ::testing::Test {
 protected:
  ComputeCostDifferentials()
      : trajectory_{create_random_traj()},
        compute_cost_{CostFunc(Q_, R_, trajectory_)} {
    compute_cost_(trajectory_.front().state, trajectory_.front().control, idx_,
                  &cost_diffs_);
  }

  Trajectory<QuadrotorModel> trajectory_;
  CostFunc::CostHessianStateState Q_ =
      CostFunc::CostHessianStateState::Identity();
  const CostFunc::CostHessianControlControl R_ =
      CostFunc::CostHessianControlControl::Identity();
  CostFunc::CostDifferentials cost_diffs_;

  double compute_cost(const State &x, const Control &u) {
    return compute_cost_(x, u, idx_);
  }

 private:
  const int idx_ = 0;
  CostFunc compute_cost_;
};

TEST_F(ComputeCostDifferentials, CostStateJacobianCloseToFiniteDifference) {
  const auto EPS = 1e-6;
  CostFunc::CostJacobianState C_x_finite_diff{};
  const auto &x = trajectory_.front().state;
  const auto &u = trajectory_.front().control;
  for (size_t i = 0; i < STATE_DIM; ++i) {
    StateTangent delta_x = StateTangent ::Zero();
    delta_x[i] = EPS;
    C_x_finite_diff(i) =
        (compute_cost(x + delta_x, u) - compute_cost(x + -1 * delta_x, u)) /
        (2 * EPS);
  }
  const auto error =
      (cost_diffs_.x - C_x_finite_diff).norm() / C_x_finite_diff.size();
  EXPECT_LT(error, EPS);
}

TEST_F(ComputeCostDifferentials, CostStateStateHessianCloseToFiniteDifference) {
  const auto EPS = 1e-6;
  CostFunc::CostHessianStateState C_xx_finite_diff{};
  const auto &x = trajectory_.front().state;
  const auto &u = trajectory_.front().control;
  for (size_t i = 0; i < STATE_DIM; ++i) {
    StateTangent delta_x_0 = StateTangent::Zero();
    delta_x_0[i] = EPS;
    for (size_t j = 0; j < STATE_DIM; ++j) {
      StateTangent delta_x_1 = StateTangent::Zero();
      delta_x_1[j] = EPS;
      C_xx_finite_diff(i, j) =
          (compute_cost(x + (delta_x_0 + delta_x_1), u) -
           compute_cost(x + (delta_x_0 - delta_x_1), u) -
           compute_cost(x + (-1 * delta_x_0 + delta_x_1), u) +
           compute_cost(x + (-1 * delta_x_0 - delta_x_1), u)) /
          (4 * EPS * EPS);
    }
  }

  const auto error = (cost_diffs_.xx.inverse() * C_xx_finite_diff -
                      CostFunc::CostHessianStateState::Identity())
                         .norm();
  EXPECT_LT(error, 11.0);
}

TEST_F(ComputeCostDifferentials, CostControlJacobianCloseToFiniteDifference) {
  const auto EPS = 1e-6;
  CostFunc::CostJacobianControl C_u_finite_diff{};
  const auto &x = trajectory_.front().state;
  const auto &u = trajectory_.front().control;
  for (size_t i = 0; i < CONTROL_DIM; ++i) {
    Control delta_u = Control::Zero();
    delta_u[i] = EPS;
    C_u_finite_diff(i) =
        (compute_cost(x, u + delta_u) - compute_cost(x, u - delta_u)) /
        (2 * EPS);
  }
  const auto error =
      (cost_diffs_.u - C_u_finite_diff).norm() / C_u_finite_diff.size();
  EXPECT_LT(error, EPS);
}

TEST_F(ComputeCostDifferentials,
       CostControlControlHessianCloseToFiniteDifference) {
  const auto EPS = 1e-6;
  CostFunc::CostHessianControlControl C_uu_finite_diff{};
  const auto &x = trajectory_.front().state;
  const auto &u = trajectory_.front().control;
  for (size_t i = 0; i < CONTROL_DIM; ++i) {
    Control delta_u_0 = Control::Zero();
    delta_u_0[i] = EPS;
    for (size_t j = 0; j < CONTROL_DIM; ++j) {
      Control delta_u_1 = Control::Zero();
      delta_u_1[j] = EPS;
      C_uu_finite_diff(i, j) =
          (compute_cost(x, u + (delta_u_0 + delta_u_1)) -
           compute_cost(x, u + (delta_u_0 - delta_u_1)) -
           compute_cost(x, u + (-1 * delta_u_0 + delta_u_1)) +
           compute_cost(x, u + (-1 * delta_u_0 - delta_u_1))) /
          (4 * EPS * EPS);
    }
  }

  const auto error = (cost_diffs_.uu.inverse() * C_uu_finite_diff -
                      CostFunc::CostHessianControlControl::Identity())
                         .norm();
  EXPECT_LT(error, 110.0);
}
}  // namespace src