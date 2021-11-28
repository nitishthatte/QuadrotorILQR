#pragma once

#include "src/cost.hh"
#include "src/dynamics.hh"
#include "src/trajectory.hh"

namespace src {

struct LineSearchParams {
  double step_update;
  double desired_reduction_frac;

  LineSearchParams(double step_update_, double desired_reduction_frac_)
      : step_update(step_update_),
        desired_reduction_frac(desired_reduction_frac_) {
    assert(step_update > 0.0);
    assert(step_update < 1.0);
    assert(desired_reduction_frac > 0.0);
    assert(desired_reduction_frac < 1.0);
  }
};

template <class ModelT>
struct ILQR {
  ILQR(CostFunction<ModelT> cost_function, LineSearchParams line_search_params)
      : cost_function_{std::move(cost_function)},
        line_search_params_{std::move(line_search_params)} {}

  using CostFunc = CostFunction<ModelT>;
  CostFunc cost_function_;
  LineSearchParams line_search_params_;

  struct OptDiffs {
    typename ModelT::DynamicsDifferentials dynamics_diffs;
    typename CostFunction<ModelT>::CostDifferentials cost_diffs;
  };

  using ControlTrajectory = std::vector<typename ModelT::Control>;

  using FeedbackGains =
      Eigen::Matrix<double, ModelT::STATE_DIM, ModelT::CONTROL_DIM>;

  struct ControlUpdate {
    typename ModelT::Control::Tangent ff_update;
    FeedbackGains feedback;
  };

  using ControlUpdateTrajectory = std::vector<ControlUpdate>;

  std::pair<Trajectory<ModelT>, double> forward_pass(
      const Trajectory<ModelT> &current_traj,
      const ControlUpdateTrajectory &ctrl_update_traj,
      const double line_search_alpha = 1.0,
      std::vector<OptDiffs> *opt_diffs_ptr = nullptr) const {
    Trajectory<ModelT> updated_traj;
    updated_traj.reserve(current_traj.size());

    auto state = current_traj.front().state;
    double cost = 0;

    for (int i = 0; i < current_traj.size(); ++i) {
      auto control =
          current_traj[i].control +
          line_search_alpha * ctrl_update_traj[i].ff_update +
          ctrl_update_traj[i].feedback * (state - current_traj[i].state);

      updated_traj.emplace_back(
          TrajectoryPoint<ModelT>{.time_s = current_traj[i].time_s,
                                  .state = state,
                                  .control = control});

      auto [dynamics_diffs_ptr, cost_diffs_ptr] =
          opt_diffs_ptr == nullptr
              ? std::make_pair(nullptr, nullptr)
              : std::make_pair(&(opt_diffs_ptr->at(i).dynamics_diffs),
                               &(opt_diffs_ptr->at(i).cost_diffs));
      cost += cost_function_(state, control, i, cost_diffs_ptr);
      state = ModelT::dynamics(state, control, dynamics_diffs_ptr);
    }

    return std::make_pair(updated_traj, cost);
  }

  std::pair<ControlUpdateTrajectory, double> backwards_pass(
      const std::vector<OptDiffs> opt_diffs) const {
    ControlUpdateTrajectory ctrl_update_traj;
    ctrl_update_traj.reserve(opt_diffs.size());

    typename CostFunc::CostJacobianState v_x =
        CostFunc::CostJacobianState::Zero();
    typename CostFunc::CostHessianStateState v_xx =
        CostFunc::CostHessianStateState::Zero();
    typename CostFunc::CostDifferentials Q;
    for (auto opt_diffs_iter = opt_diffs.crbegin();
         opt_diffs_iter != opt_diffs.crend(); ++opt_diffs_iter) {
      Q = typename CostFunc::CostDifferentials{
          .x = opt_diffs_iter->cost_diffs.x +
               opt_diffs_iter->dynamics_diffs.J_x.transpose() * v_x,
          .u = opt_diffs_iter->cost_diffs.u +
               opt_diffs_iter->dynamics_diffs.J_u.transpose() * v_x,
          .xx = opt_diffs_iter->cost_diffs.xx +
                opt_diffs_iter->dynamics_diffs.J_x.transpose() * v_xx *
                    opt_diffs_iter->dynamics_diffs.J_x,
          .xu = opt_diffs_iter->cost_diffs.xu +
                opt_diffs_iter->dynamics_diffs.J_x.transpose() * v_xx *
                    opt_diffs_iter->dynamics_diffs.J_u,
          .uu = opt_diffs_iter->cost_diffs.uu +
                opt_diffs_iter->dynamics_diffs.J_u.transpose() * v_xx *
                    opt_diffs_iter->dynamics_diffs.J_u,
      };

      const auto Quu_ldlt = Q.uu.ldlt();
      ctrl_update_traj.emplace_back(
          ControlUpdate{.ff_update = -Quu_ldlt.solve(Q.u),
                        .feedback = -Quu_ldlt.solve(Q.xu.transpose())});

      v_x = opt_diffs_iter->cost_diffs.x -
            ctrl_update_traj.back().feedback.transpose() *
                opt_diffs_iter->cost_diffs.uu *
                ctrl_update_traj.back().ff_update;
      v_xx = opt_diffs_iter->cost_diffs.xx -
             ctrl_update_traj.back().feedback.transpose() *
                 opt_diffs_iter->cost_diffs.uu *
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

  double line_search(const ControlUpdateTrajectory &ctrl_update_traj,
                     const double expected_cost_reduction) const {}
};

}  // namespace src