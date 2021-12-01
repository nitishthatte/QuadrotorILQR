#pragma once

#include <stdexcept>
#include <string>

#include "src/cost.hh"
#include "src/dynamics.hh"
#include "src/trajectory.hh"

namespace src {

struct LineSearchParams {
  double step_update;
  double desired_reduction_frac;
  int max_iters;

  LineSearchParams(const double step_update_,
                   const double desired_reduction_frac_, const int max_iters_)
      : step_update(step_update_),
        desired_reduction_frac(desired_reduction_frac_),
        max_iters(max_iters_) {
    assert(step_update > 0.0);
    assert(step_update < 1.0);
    assert(desired_reduction_frac > 0.0);
    assert(desired_reduction_frac < 1.0);
    assert(max_iters > 0);
  }
};

struct ConvergenceCriteria {
  double rtol;
  double atol;
  double max_iters;
};

template <class ModelT>
struct ILQR {
  ILQR(CostFunction<ModelT> cost_function, LineSearchParams line_search_params)
      : cost_function_{std::move(cost_function)},
        line_search_params_{std::move(line_search_params)} {}

  using CostFunc = CostFunction<ModelT>;
  CostFunc cost_function_;
  LineSearchParams line_search_params_;

  using DynamicsDiffs = typename ModelT::DynamicsDifferentials;
  using CostDiffs = typename CostFunction<ModelT>::CostDifferentials;

  using FeedbackGains =
      Eigen::Matrix<double, ModelT::STATE_DIM, ModelT::CONTROL_DIM>;

  struct ControlUpdate {
    typename ModelT::Control::Tangent ff_update;
    FeedbackGains feedback;
  };

  using ControlUpdateTrajectory = std::vector<ControlUpdate>;

  Trajectory<ModelT> forward_pass(
      const Trajectory<ModelT> &current_traj,
      const ControlUpdateTrajectory &ctrl_update_traj,
      const double line_search_alpha = 1.0,
      std::vector<DynamicsDiffs> *diffs_ptr = nullptr) const {
    Trajectory<ModelT> updated_traj;
    updated_traj.reserve(current_traj.size());

    auto state = current_traj.front().state;
    for (int i = 0; i < current_traj.size(); ++i) {
      auto control =
          current_traj[i].control +
          line_search_alpha * ctrl_update_traj[i].ff_update +
          ctrl_update_traj[i].feedback * (state - current_traj[i].state);

      updated_traj.emplace_back(
          TrajectoryPoint<ModelT>{.time_s = current_traj[i].time_s,
                                  .state = state,
                                  .control = control});

      auto diff_ptr = diffs_ptr == nullptr ? nullptr : &diffs_ptr->at(i);
      state = ModelT::dynamics(state, control, diff_ptr);
    }

    return updated_traj;
  }

  double cost_trajectory(const Trajectory<ModelT> &traj,
                         std::vector<CostDiffs> *diffs_ptrs = nullptr) const {
    auto cost = 0.0;
    for (int i = 0; i < traj.size(); ++i) {
      auto diff_ptr = diffs_ptrs == nullptr ? nullptr : &diffs_ptrs->at(i);
      cost += cost_function_(traj[i].state, traj[i].control, i, diff_ptr);
    }
    return cost;
  }

  std::pair<ControlUpdateTrajectory, double> backwards_pass(
      const std::vector<DynamicsDiffs> &dynamics_diffs,
      const std::vector<CostDiffs> &cost_diffs) const {
    ControlUpdateTrajectory ctrl_update_traj;
    assert(dynamics_diffs.size() == cost_diffs.size());
    const auto num_pts = cost_diffs.size();
    ctrl_update_traj.reserve(cost_diffs.size());

    typename CostFunc::CostJacobianState v_x =
        CostFunc::CostJacobianState::Zero();
    typename CostFunc::CostHessianStateState v_xx =
        CostFunc::CostHessianStateState::Zero();
    typename CostFunc::CostDifferentials Q;
    for (int i = num_pts - 1; i >= 0; --i) {
      Q = typename CostFunc::CostDifferentials{
          .x = cost_diffs[i].x + dynamics_diffs[i].J_x.transpose() * v_x,
          .u = cost_diffs[i].u + dynamics_diffs[i].J_u.transpose() * v_x,
          .xx = cost_diffs[i].xx + dynamics_diffs[i].J_x.transpose() * v_xx *
                                       dynamics_diffs[i].J_x,
          .xu = cost_diffs[i].xu + dynamics_diffs[i].J_x.transpose() * v_xx *
                                       dynamics_diffs[i].J_u,
          .uu = cost_diffs[i].uu + dynamics_diffs[i].J_u.transpose() * v_xx *
                                       dynamics_diffs[i].J_u,
      };

      const auto Quu_ldlt = Q.uu.ldlt();
      ctrl_update_traj.emplace_back(
          ControlUpdate{.ff_update = -Quu_ldlt.solve(Q.u),
                        .feedback = -Quu_ldlt.solve(Q.xu.transpose())});

      v_x = cost_diffs[i].x - ctrl_update_traj.back().feedback.transpose() *
                                  cost_diffs[i].uu *
                                  ctrl_update_traj.back().ff_update;
      v_xx = cost_diffs[i].xx - ctrl_update_traj.back().feedback.transpose() *
                                    cost_diffs[i].uu *
                                    ctrl_update_traj.back().feedback;
    }

    std::reverse(ctrl_update_traj.begin(), ctrl_update_traj.end());

    // expected value reduction
    const typename ModelT::Control::Tangent &ff_update =
        ctrl_update_traj.front().ff_update;
    const double delta_v =
        (Q.u.transpose() * ff_update.coeffs() +
         0.5 * ff_update.coeffs().transpose() * Q.uu * ff_update.coeffs())(0);

    return std::make_pair(ctrl_update_traj, delta_v);
  }

  std::tuple<Trajectory<ModelT>, double, double> line_search(
      const Trajectory<ModelT> &current_traj, const double current_cost,
      const ControlUpdateTrajectory &ctrl_update_traj,
      const double expected_cost_reduction) const {
    assert(expected_cost_reduction <= 0);
    const auto desired_cost_reduction =
        line_search_params_.desired_reduction_frac * expected_cost_reduction;

    auto step = 1.0;
    for (int i = 0; i < line_search_params_.max_iters; ++i) {
      const auto new_traj = forward_pass(current_traj, ctrl_update_traj, step);
      const auto new_cost = cost_trajectory(new_traj);
      if (new_cost - current_cost < step * desired_cost_reduction) {
        return std::make_tuple(new_traj, new_cost, step);
      }
      step *= line_search_params_.step_update;
    }
    throw std::runtime_error(
        "Reached maximum number of line search iterations, " +
        std::to_string(line_search_params_.max_iters) + "\n");
  }

  Trajectory<ModelT> solve(const Trajectory<ModelT> &current_traj) {}
};

}  // namespace src