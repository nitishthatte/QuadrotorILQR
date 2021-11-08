#include "src/dynamics.hh"

namespace model {
manif::SE3d dynamics(const manif::SE3d &x, const manif::SE3d &u) { return x * u; }
}  // namespace model