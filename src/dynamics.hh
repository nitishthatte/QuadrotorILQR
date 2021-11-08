#pragma once
#include <manif/manif.h>

namespace model {
manif::SE3d dynamics(const manif::SE3d &x, const manif::SE3d &u);
}