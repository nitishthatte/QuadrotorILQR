#include "src/ilqr_options_to_proto.hh"

#include <gtest/gtest.h>

namespace src {

TEST(ILQROptionsToProto, RoundTripTest) {
  const ILQROptions options{
      .line_search_params = {.step_update = 1.0,
                             .desired_reduction_frac = 2.0,
                             .max_iters = 3},
      .convergence_criteria = {.rtol = 4.0, .atol = 5.0, .max_iters = 6}};

  const auto round_trip = from_proto(to_proto(options));

  EXPECT_EQ(options, round_trip);
}
}  // namespace src