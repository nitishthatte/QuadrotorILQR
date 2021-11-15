#pragma once

#include "src/cost.hh"
#include "src/dynamics.hh"
#include "src/trajectory.hh"

namespace src {

template <class ModelT>
struct ILQR {
  ILQR(CostFunction<ModelT> cost_function)
      : cost_function_{std::move(cost_function)} {}

  CostFunction<ModelT> cost_function_;

  using FeedbackGains =
      Eigen::Matrix<double, ModelT::STATE_DIM, ModelT::CONTROL_DIM>;

  struct OptDiffs {
    typename ModelT::DynamicsDifferentials dynamics_diffs;
    typename CostFunction<ModelT>::CostDifferentials cost_diffs;
  };

  std::pair<Trajectory<ModelT>, double> forward_pass(
      const Trajectory<ModelT> &current_traj,
      const std::vector<typename ModelT::Control::Tangent> &delta_u_ff,
      const std::vector<FeedbackGains> &gains,
      const double line_search_alpha = 1.0,
      OptDiffs *opt_diffs_ptr = nullptr) const {
    Trajectory<ModelT> updated_traj;
    updated_traj.reserve(current_traj.size());

    auto state = current_traj.front().state;
    for (int i = 0; i < current_traj.size(); ++i) {
      auto control = current_traj[i].control +
                     line_search_alpha * delta_u_ff[i] +
                     gains[i] * (state - current_traj[i].state);

      updated_traj.emplace_back(
          TrajectoryPoint<ModelT>{.time_s = current_traj[i].time_s,
                                  .state = state,
                                  .control = control});
      state = ModelT::dynamics(state, control);
    }

    return std::make_pair(updated_traj, 0.0);
  }
};
}  // namespace src