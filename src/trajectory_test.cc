#include "trajectory.hh"

#include <gtest/gtest.h>

namespace src {

using State = LieDynamics::State;
using Control = LieDynamics::Control;

TEST(TrajectoryPoint, EqualityOperatorReturnsTrueIfTrajectoriesSame) {
  const auto pt = TrajectoryPoint<LieDynamics>{.time_s = 0.0,
                                               .state = State::Identity(),
                                               .control = Control::Identity()};

  EXPECT_TRUE(pt == pt);
}
}  // namespace src