#include "trajectory.hh"

#include <gtest/gtest.h>

#include "dynamics.hh"

namespace src {

using State = LieDynamics::State;
using Control = LieDynamics::Control;

TEST(TrajectoryPoint, EqualityOperatorReturnsTrueIfTrajectoryPointsSame) {
  const auto pt = TrajectoryPoint<LieDynamics>{.time_s = 0.0,
                                               .state = State::Identity(),
                                               .control = Control::Identity()};

  EXPECT_TRUE(pt == pt);
}

TEST(TrajectoryPoint,
     EqualityOperatorReturnsFalseIfTrajectoryPointsTimeDifferent) {
  const auto pt0 = TrajectoryPoint<LieDynamics>{.time_s = 0.0,
                                                .state = State::Identity(),
                                                .control = Control::Identity()};
  const auto pt1 = TrajectoryPoint<LieDynamics>{.time_s = 1.0,
                                                .state = State::Identity(),
                                                .control = Control::Identity()};

  EXPECT_FALSE(pt0 == pt1);
}

TEST(TrajectoryPoint,
     EqualityOperatorReturnsFalseIfTrajectoryPointsStateDifferent) {
  const auto pt0 = TrajectoryPoint<LieDynamics>{.time_s = 0.0,
                                                .state = State::Identity(),
                                                .control = Control::Identity()};
  const auto pt1 = TrajectoryPoint<LieDynamics>{
      .time_s = 1.0,
      .state = State{{1.0, 0.0, 0.0}, manif::SO3d::Identity()},
      .control = Control::Identity()};

  EXPECT_FALSE(pt0 == pt1);
}

TEST(TrajectoryPoint,
     EqualityOperatorReturnsFalseIfTrajectoryPointsControlDifferent) {
  const auto pt0 = TrajectoryPoint<LieDynamics>{.time_s = 0.0,
                                                .state = State::Identity(),
                                                .control = Control::Identity()};
  const auto pt1 = TrajectoryPoint<LieDynamics>{
      .time_s = 1.0,
      .state = State::Identity(),
      .control = Control{{1.0, 0.0, 0.0}, manif::SO3d::Identity()}};

  EXPECT_FALSE(pt0 == pt1);
}
}  // namespace src