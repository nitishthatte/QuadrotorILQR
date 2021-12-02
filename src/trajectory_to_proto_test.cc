#include "trajectory_to_proto.hh"

#include <gtest/gtest.h>

namespace src {

using State = LieDynamics::State;
using Control = LieDynamics::Control;

TEST(LieTrajectory, RoundTripTest) {
  Trajectory<LieDynamics> traj{
      {.time_s = 1.0,
       .state = manif::SE3d{1.0, 2.0, 3.0, 4.0, 5.0, 6.0},
       .control = manif::SE3d{2.0, 3.0, 4.0, 5.0, 6.0, 7.0}},
      {.time_s = 2.0,
       .state = manif::SE3d{4.0, 5.0, 6.0, 7.0, 8.0, 9.0},
       .control = manif::SE3d{15.0, 14.0, 13.0, 12.0, 11.0, 10.0}},
  };

  const auto round_trip = from_proto(to_proto(traj));

  EXPECT_EQ(traj, round_trip);
}
}  // namespace src