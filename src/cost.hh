#pragma once
#include <manif/manif.h>

#include <vector>

#include "src/dynamics.hh"

namespace src {

using CostJacobianState = Eigen::Matrix<double, StateDim, 1>;
using CostJacobianControl = Eigen::Matrix<double, ControlDim, 1>;
using CostHessianStateState = Eigen::Matrix<double, StateDim, StateDim>;
using CostHessianControlControl = Eigen::Matrix<double, ControlDim, ControlDim>;
using CostHessianStateControl = Eigen::Matrix<double, StateDim, ControlDim>;

struct CostDifferentials {
  CostJacobianState C_x;
  CostJacobianControl C_u;
  CostHessianStateState C_xx;
  CostHessianControlControl C_uu;
  CostHessianStateControl C_xu;
};

class CostFunction {
 public:
  CostFunction(CostHessianStateState Q, CostHessianControlControl R,
               std::vector<manif::SE3d> desired_states,
               std::vector<manif::SE3d> desired_controls)
      : Q_{std::move(Q)},
        R_{std::move(R)},
        desired_states_{std::move(desired_states)},
        desired_controls_{std::move(desired_controls)} {}

  double operator()(const manif::SE3d &x, const manif::SE3d &u, const int i,
                    CostDifferentials *diffs = nullptr) const;

 private:
  CostHessianStateState Q_;
  CostHessianControlControl R_;
  std::vector<manif::SE3d> desired_states_;
  std::vector<manif::SE3d> desired_controls_;
};
}  // namespace src