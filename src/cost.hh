#pragma once
#include <manif/manif.h>

#include <vector>

#include "src/trajectory.hh"

namespace src {

template <class ModelT>
class CostFunction {
 public:
  using CostJacobianState = Eigen::Matrix<double, ModelT::STATE_DIM, 1>;
  using CostJacobianControl = Eigen::Matrix<double, ModelT::CONTROL_DIM, 1>;
  using CostHessianStateState =
      Eigen::Matrix<double, ModelT::STATE_DIM, ModelT::STATE_DIM>;
  using CostHessianControlControl =
      Eigen::Matrix<double, ModelT::CONTROL_DIM, ModelT::CONTROL_DIM>;
  using CostHessianStateControl =
      Eigen::Matrix<double, ModelT::STATE_DIM, ModelT::CONTROL_DIM>;

  struct CostDifferentials {
    CostJacobianState x;
    CostJacobianControl u;
    CostHessianStateState xx;
    CostHessianControlControl uu;
    CostHessianStateControl xu;
  };

  CostFunction(CostHessianStateState Q, CostHessianControlControl R,
               Trajectory<ModelT> desired_trajectory)
      : Q_{std::move(Q)},
        R_{std::move(R)},
        desired_trajectory_{std::move(desired_trajectory)} {}

  double operator()(const typename ModelT::State &x,
                    const typename ModelT::Control &u, const int i,
                    CostDifferentials *diffs = nullptr) const {
    const auto &x_d = desired_trajectory_.at(i).state;
    const auto &u_d = desired_trajectory_.at(i).control;

    typename ModelT::BinaryStateFuncDiffs minus_diffs;
    const auto delta_x = minus(x, x_d, &minus_diffs);
    const typename ModelT::StateJacobian &J_delta_x = minus_diffs.J_x_lhs;
    const auto delta_u = u - u_d;

    const auto cost = (delta_x.coeffs().transpose() * Q_ * delta_x.coeffs() +
                       delta_u.transpose() * R_ * delta_u)(0);

    if (diffs) {
      diffs->x = 2 * delta_x.coeffs().transpose() * Q_ * J_delta_x;
      diffs->xx = 2 * J_delta_x.transpose() * Q_ * J_delta_x;

      diffs->u = 2 * delta_u.transpose() * R_;
      diffs->uu = 2 * R_;

      diffs->xu = CostHessianStateControl::Zero();
    }

    return cost;
  }

 private:
  CostHessianStateState Q_;
  CostHessianControlControl R_;
  Trajectory<ModelT> desired_trajectory_;
};
}  // namespace src