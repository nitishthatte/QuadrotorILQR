#include "src/ilqr_options.hh"

namespace src {

bool operator==(const LineSearchParams &lhs, const LineSearchParams &rhs) {
  return lhs.step_update == rhs.step_update &&
         lhs.desired_reduction_frac == rhs.desired_reduction_frac &&
         lhs.max_iters == rhs.max_iters;
}

bool operator==(const ConvergenceCriteria &lhs,
                const ConvergenceCriteria &rhs) {
  return lhs.rtol == rhs.rtol && lhs.atol == rhs.atol &&
         lhs.max_iters == rhs.max_iters;
}

bool operator==(const ILQROptions &lhs, const ILQROptions &rhs) {
  return lhs.line_search_params == rhs.line_search_params &&
         lhs.convergence_criteria == rhs.convergence_criteria;
}
}  // namespace src