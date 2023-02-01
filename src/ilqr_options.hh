#pragma once
namespace src {

struct LineSearchParams {
  double step_update;
  double desired_reduction_frac;
  int max_iters;
};
bool operator==(const LineSearchParams &lhs, const LineSearchParams &rhs);

struct ConvergenceCriteria {
  double rtol;
  double atol;
  double max_iters;
};
bool operator==(const ConvergenceCriteria &lhs, const ConvergenceCriteria &rhs);

struct ILQROptions {
  LineSearchParams line_search_params;
  ConvergenceCriteria convergence_criteria;
  bool populate_debug;
};
bool operator==(const ILQROptions &lhs, const ILQROptions &rhs);
}  // namespace src