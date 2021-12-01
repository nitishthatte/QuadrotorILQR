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
  ILQR(CostFunction<ModelT> cost_function, LineSearchParams line_search_params,
       ConvergenceCriteria convergence_criteria)
      : cost_function_{std::move(cost_function)},
        line_search_params_{std::move(line_search_params)},
        convergence_criteria_{std::move(convergence_criteria)} {}

  using CostFunc = CostFunction<ModelT>;
  CostFunc cost_function_;
  LineSearchParams line_search_params_;
  ConvergenceCriteria convergence_criteria_;

  using DynamicsDiffs = typename ModelT::DynamicsDifferentials;
  using CostDiffs = typename CostFunction<ModelT>::CostDifferentials;

  using FeedbackGains =
      Eigen::Matrix<double, ModelT::STATE_DIM, ModelT::CONTROL_DIM>;

  struct ControlUpdate {
    typename ModelT::Control::Tangent ff_update;
    FeedbackGains feedback;
  };

  using ControlUpdateTrajectory = std::vector<ControlUpdate>;

  Trajectory<ModelT> forward_sim(
      const Trajectory<ModelT> &current_traj,
      const ControlUpdateTrajectory &ctrl_update_traj,
      const double line_search_alpha = 1.0) const {
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

      state = ModelT::dynamics(state, control);
    }

    return updated_traj;
  }

  double cost_trajectory(const Trajectory<ModelT> &traj) const {
    auto cost = 0.0;
    for (int i = 0; i < traj.size(); ++i) {
      cost += cost_function_(traj[i].state, traj[i].control, i);
    }
    return cost;
  }

  std::pair<ControlUpdateTrajectory, double> backwards_pass(
      const Trajectory<ModelT> &traj) const {
    ControlUpdateTrajectory ctrl_update_traj;
    const auto num_pts = traj.size();
    ctrl_update_traj.reserve(num_pts);

    typename CostFunc::CostJacobianState v_x =
        CostFunc::CostJacobianState::Zero();
    typename CostFunc::CostHessianStateState v_xx =
        CostFunc::CostHessianStateState::Zero();
    typename CostFunc::CostDifferentials Q;
    for (int i = num_pts - 1; i >= 0; --i) {
      DynamicsDiffs dynamics_diffs;
      ModelT::dynamics(traj[i].state, traj[i].control, &dynamics_diffs);
      const auto &[J_x, J_u] = dynamics_diffs;

      CostDiffs C;
      cost_function_(traj[i].state, traj[i].control, i, &C);

      Q = typename CostFunc::CostDifferentials{
          .x = C.x + J_x.transpose() * v_x,
          .u = C.u + J_u.transpose() * v_x,
          .xx = C.xx + J_x.transpose() * v_xx * J_x,
          .xu = C.xu + J_x.transpose() * v_xx * J_u,
          .uu = C.uu + J_u.transpose() * v_xx * J_u,
      };

      const auto Quu_ldlt = Q.uu.ldlt();
      const FeedbackGains K = -Quu_ldlt.solve(Q.xu.transpose());
      const typename ModelT::Control::Tangent k = -Quu_ldlt.solve(Q.u);
      ctrl_update_traj.emplace_back(
          ControlUpdate{.ff_update = k, .feedback = K});

      v_x = C.x - K.transpose() * C.uu * k;
      v_xx = C.xx - K.transpose() * C.uu * K;
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
      const auto new_traj = forward_sim(current_traj, ctrl_update_traj, step);
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

  Trajectory<ModelT> solve(Trajectory<ModelT> traj) {
    auto new_cost = cost_trajectory(traj);
    for (int i = 0; i < convergence_criteria_.max_iters; ++i) {
      const auto [ctrl_update_traj, expected_cost_reduction] =
          backwards_pass(traj);
      const double cost = new_cost;
      std::tie(traj, new_cost, std::ignore) =
          line_search(traj, cost, ctrl_update_traj, expected_cost_reduction);

      if (is_converged(cost, new_cost)) {
        return traj;
      }
      std::cerr << i << std::endl;
    }
    return traj;
  }

  bool is_converged(const double cost, const double new_cost) {
    if (std::abs(cost - new_cost) / std::abs(cost) <
        convergence_criteria_.rtol) {
      return true;
    }
    if (std::abs(cost - new_cost) < convergence_criteria_.atol) {
      return true;
    }
    return false;
  }
};

}  // namespace src