#pragma once

#include <stdexcept>
#include <string>

#include "src/cost.hh"
#include "src/ilqr_options.hh"
#include "src/trajectory.hh"

namespace src {
namespace detail {
struct CostReductionTerms {
  double QuTk = 0;
  double kTQuuk = 0;
};

inline double calculate_cost_reduction(
    const CostReductionTerms &cost_reduction_terms, const double step = 1.0) {
  return step * cost_reduction_terms.QuTk +
         step * step * cost_reduction_terms.kTQuuk / 2.0;
}
}  // namespace detail

template <class ModelT>
struct ILQR {
  ILQR(ModelT model, CostFunction<ModelT> cost_function, double dt_s,
       ILQROptions options)
      : model_{std::move(model)},
        cost_function_{std::move(cost_function)},
        dt_s_{dt_s},
        options_{std::move(options)} {}

  ModelT model_;
  using CostFunc = CostFunction<ModelT>;
  CostFunc cost_function_;
  double dt_s_;
  ILQROptions options_;

  using DynamicsDiffs = typename ModelT::DynamicsDifferentials;
  using CostDiffs = typename CostFunction<ModelT>::CostDifferentials;

  using FeedbackGains =
      Eigen::Matrix<double, ModelT::CONTROL_DIM, ModelT::STATE_DIM>;

  struct ControlUpdate {
    typename ModelT::Control ff_update;
    FeedbackGains feedback;
  };

  using ControlUpdateTrajectory = std::vector<ControlUpdate>;

  Trajectory<ModelT> solve(const Trajectory<ModelT> &initial_traj) const {
    auto traj = initial_traj;
    auto new_cost = cost_trajectory(traj);
    for (int i = 0; i < options_.convergence_criteria.max_iters; ++i) {
      const auto [ctrl_update_traj, cost_reduction_terms] =
          backwards_pass(traj);
      const double cost = new_cost;

      // if expected cost reduction of this line search is small, return
      const auto expected_new_cost =
          cost + detail::calculate_cost_reduction(cost_reduction_terms);
      if (i > 0 && is_converged(cost, expected_new_cost)) {
        return traj;
      }

      if (i == 0) {
        traj = forward_sim(traj, ctrl_update_traj, 1.0);
        new_cost = cost_trajectory(traj);
      } else {
        std::tie(traj, new_cost, std::ignore) =
            line_search(traj, cost, ctrl_update_traj, cost_reduction_terms);
      }

      if (i > 0 && is_converged(cost, new_cost)) {
        return traj;
      }
    }
    return traj;
  }

  double cost_trajectory(const Trajectory<ModelT> &traj) const {
    auto cost = 0.0;
    for (int i = 0; i < traj.size(); ++i) {
      cost += cost_function_(traj[i].state, traj[i].control, i);
    }
    return cost;
  }

  std::pair<ControlUpdateTrajectory, detail::CostReductionTerms> backwards_pass(
      const Trajectory<ModelT> &traj) const {
    ControlUpdateTrajectory ctrl_update_traj;
    const auto num_pts = traj.size();
    ctrl_update_traj.reserve(num_pts);

    typename CostFunc::CostJacobianState v_x =
        CostFunc::CostJacobianState::Zero();
    typename CostFunc::CostHessianStateState v_xx =
        CostFunc::CostHessianStateState::Zero();
    typename CostFunc::CostDifferentials Q;
    detail::CostReductionTerms cost_reduction_terms{};
    for (int i = num_pts - 1; i >= 0; --i) {
      DynamicsDiffs dynamics_diffs;
      model_.discrete_dynamics(traj[i].state, traj[i].control, dt_s_,
                               &dynamics_diffs);
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
      const typename ModelT::Control k = -Quu_ldlt.solve(Q.u);
      ctrl_update_traj.emplace_back(
          ControlUpdate{.ff_update = k, .feedback = K});

      v_x = Q.x - K.transpose() * Q.uu * k;
      v_xx = Q.xx - K.transpose() * Q.uu * K;

      // expected cost reduction
      const typename ModelT::Control &ff_update =
          ctrl_update_traj.back().ff_update;
      cost_reduction_terms.QuTk += (Q.u.transpose() * ff_update)(0);
      cost_reduction_terms.kTQuuk +=
          (ff_update.transpose() * Q.uu * ff_update)(0);
    }

    std::reverse(ctrl_update_traj.begin(), ctrl_update_traj.end());

    return std::make_pair(std::move(ctrl_update_traj),
                          std::move(cost_reduction_terms));
  }

  Trajectory<ModelT> forward_sim(
      const Trajectory<ModelT> &current_traj,
      const ControlUpdateTrajectory &ctrl_update_traj,
      const double line_search_alpha = 1.0) const {
    Trajectory<ModelT> updated_traj;
    updated_traj.reserve(current_traj.size());

    auto state = current_traj.front().state;
    for (int i = 0; i < current_traj.size(); ++i) {
      auto control = current_traj[i].control +
                     line_search_alpha * ctrl_update_traj[i].ff_update +
                     ctrl_update_traj[i].feedback *
                         (state - current_traj[i].state).coeffs();

      updated_traj.emplace_back(
          TrajectoryPoint<ModelT>{.time_s = current_traj[i].time_s,
                                  .state = state,
                                  .control = control});

      state = model_.discrete_dynamics(state, control, dt_s_);
    }

    return updated_traj;
  }

  std::tuple<Trajectory<ModelT>, double, double> line_search(
      const Trajectory<ModelT> &current_traj, const double current_cost,
      const ControlUpdateTrajectory &ctrl_update_traj,
      const detail::CostReductionTerms &cost_reduction_terms) const {
    auto step = 1.0;
    for (int i = 0; i < options_.line_search_params.max_iters; ++i) {
      const auto new_traj = forward_sim(current_traj, ctrl_update_traj, step);
      const auto new_cost = cost_trajectory(new_traj);
      const auto desired_cost_reduction =
          options_.line_search_params.desired_reduction_frac *
          detail::calculate_cost_reduction(cost_reduction_terms, step);

      if (new_cost - current_cost < desired_cost_reduction) {
        return std::make_tuple(new_traj, new_cost, step);
      }
      step *= options_.line_search_params.step_update;
    }
    throw std::runtime_error(
        "Reached maximum number of line search iterations, " +
        std::to_string(options_.line_search_params.max_iters) + "\n");
  }

  bool is_converged(const double cost, const double new_cost) const {
    if (std::abs(cost - new_cost) / std::abs(cost) <
        options_.convergence_criteria.rtol) {
      return true;
    }
    if (std::abs(cost - new_cost) < options_.convergence_criteria.atol) {
      return true;
    }
    return false;
  }
};

}  // namespace src