#pragma once
#include <manif/manif.h>

#include <vector>

#include "src/dynamics.hh"
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

    typename ModelT::State::Tangent::Jacobian J_delta_x, J_delta_u;
    const auto delta_x = x.minus(x_d, J_delta_x);
    const auto delta_u = u.minus(u_d, J_delta_u);

    const auto cost = (delta_x.coeffs().transpose() * Q_ * delta_x.coeffs() +
                       delta_u.coeffs().transpose() * R_ * delta_u.coeffs())(0);

    if (diffs) {
      diffs->x = 2 * delta_x.coeffs().transpose() * Q_ * J_delta_x;
      diffs->xx = 2 * J_delta_x.transpose() * Q_ * J_delta_x;

      diffs->u = 2 * delta_u.coeffs().transpose() * R_ * J_delta_u;
      diffs->uu = 2 * J_delta_u.transpose() * R_ * J_delta_u;

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