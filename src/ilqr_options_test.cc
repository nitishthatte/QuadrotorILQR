#include "ilqr_options.hh"

#include <gtest/gtest.h>

namespace src {

TEST(LineSearchParamsEqualityOperator, TrueIfAllParamsEqual) {
  LineSearchParams params{
      .step_update = 1.0, .desired_reduction_frac = 2.0, .max_iters = 3};
  EXPECT_EQ(params, params);
}

TEST(LineSearchParamsEqualityOperator, FalseIfStepUpdateNotEqual) {
  LineSearchParams params_0{
      .step_update = 0.0, .desired_reduction_frac = 2.0, .max_iters = 3};
  LineSearchParams params_1{
      .step_update = 1.0, .desired_reduction_frac = 2.0, .max_iters = 3};
  EXPECT_NE(params_0, params_1);
}
}  // namespace src