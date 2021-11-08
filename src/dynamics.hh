#pragma once
#include <manif/manif.h>

namespace model {
manif::SE3d dynamics(const manif::SE3d &x, const manif::SE3d &u,
                     manif::SE3d::Jacobian *J_x = nullptr,
                     manif::SE3d::Jacobian *J_u = nullptr);
}