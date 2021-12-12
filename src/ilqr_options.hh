#pragma once

namespace src {

struct LineSearchParams {
  double step_update;
  double desired_reduction_frac;
  int max_iters;

  LineSearchParams(const double step_update_,
                   const double desired_reduction_frac_, const int max_iters_)
      : step_update(step_update_),
        desired_reduction_frac(desired_reduction_frac_),
        max_iters(max_iters_) {
    assert(step_update > 0.0);
    assert(step_update < 1.0);
    assert(desired_reduction_frac > 0.0);
    assert(desired_reduction_frac < 1.0);
    assert(max_iters > 0);
  }
};

struct ConvergenceCriteria {
  double rtol;
  double atol;
  double max_iters;
};
}  // namespace src