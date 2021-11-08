#pragma once
#include <manif/manif.h>

namespace src {
constexpr int StateDim = manif::SE3d::DoF;
constexpr int ControlDim = manif::SE3d::DoF;

manif::SE3d dynamics(const manif::SE3d &x, const manif::SE3d &u,
                     manif::SE3d::Jacobian *J_x = nullptr,
                     manif::SE3d::Jacobian *J_u = nullptr);
}  // namespace src