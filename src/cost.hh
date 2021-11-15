#pragma once
#include <manif/manif.h>

#include <vector>

#include "src/dynamics.hh"

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
    CostJacobianState C_x;
    CostJacobianControl C_u;
    CostHessianStateState C_xx;
    CostHessianControlControl C_uu;
    CostHessianStateControl C_xu;
  };

  CostFunction(CostHessianStateState Q, CostHessianControlControl R,
               std::vector<typename ModelT::State> desired_states,
               std::vector<typename ModelT::Control> desired_controls)
      : Q_{std::move(Q)},
        R_{std::move(R)},
        desired_states_{std::move(desired_states)},
        desired_controls_{std::move(desired_controls)} {}

  double operator()(const typename ModelT::State &x,
                    const typename ModelT::Control &u, const int i,
                    CostDifferentials *diffs = nullptr) const {
    const auto &x_d = desired_states_.at(i);
    const auto &u_d = desired_controls_.at(i);

    typename ModelT::State::Tangent::Jacobian J_delta_x, J_delta_u;
    const auto delta_x = x.minus(x_d, J_delta_x);
    const auto delta_u = u.minus(u_d, J_delta_u);

    const auto cost = (delta_x.coeffs().transpose() * Q_ * delta_x.coeffs() +
                       delta_u.coeffs().transpose() * R_ * delta_u.coeffs())(0);

    if (diffs) {
      diffs->C_x = 2 * delta_x.coeffs().transpose() * Q_ * J_delta_x;
      diffs->C_xx = 2 * J_delta_x.transpose() * Q_ * J_delta_x;

      diffs->C_u = 2 * delta_u.coeffs().transpose() * R_ * J_delta_u;
      diffs->C_uu = 2 * J_delta_u.transpose() * R_ * J_delta_u;

      diffs->C_xu = CostHessianStateControl::Zero();
    }

    return cost;
  }

 private:
  CostHessianStateState Q_;
  CostHessianControlControl R_;
  std::vector<typename ModelT::State> desired_states_;
  std::vector<typename ModelT::Control> desired_controls_;
};
}  // namespace src