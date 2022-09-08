#include "src/trajectory.hh"

#include <gtest/gtest.h>

#include "src/quadrotor_model.hh"

namespace src {

using State = QuadrotorModel::State;
using Control = QuadrotorModel::Control;

namespace {
State create_identity_state() {
  return State{.inertial_from_body = manif::SE3d::Identity(),
               .body_velocity = manif::SE3Tangentd::Zero()};
}
}  // namespace

TEST(TrajectoryPoint, EqualityOperatorReturnsTrueIfTrajectoryPointsSame) {
  const auto pt =
      TrajectoryPoint<QuadrotorModel>{.time_s = 0.0,
                                      .state = create_identity_state(),
                                      .control = Control::Zero()};

  EXPECT_TRUE(pt == pt);
}

TEST(TrajectoryPoint,
     EqualityOperatorReturnsFalseIfTrajectoryPointsTimeDifferent) {
  const auto pt0 =
      TrajectoryPoint<QuadrotorModel>{.time_s = 0.0,
                                      .state = create_identity_state(),
                                      .control = Control::Zero()};
  const auto pt1 =
      TrajectoryPoint<QuadrotorModel>{.time_s = 1.0,
                                      .state = create_identity_state(),
                                      .control = Control::Zero()};

  EXPECT_FALSE(pt0 == pt1);
}

TEST(TrajectoryPoint,
     EqualityOperatorReturnsFalseIfTrajectoryPointsStateDifferent) {
  const auto state_0 = create_identity_state();
  const auto pt0 = TrajectoryPoint<QuadrotorModel>{
      .time_s = 0.0, .state = state_0, .control = Control::Zero()};

  auto state_1 = create_identity_state();
  manif::SE3Tangentd state_1_offset{};
  state_1_offset.coeffs()(2) = 1.0;
  state_1.inertial_from_body += state_1_offset;
  const auto pt1 = TrajectoryPoint<QuadrotorModel>{
      .time_s = 1.0, .state = state_1, .control = Control::Zero()};

  EXPECT_FALSE(pt0 == pt1);
}

TEST(TrajectoryPoint,
     EqualityOperatorReturnsFalseIfTrajectoryPointsControlDifferent) {
  const auto pt0 =
      TrajectoryPoint<QuadrotorModel>{.time_s = 0.0,
                                      .state = create_identity_state(),
                                      .control = Control::Zero()};
  const auto pt1 =
      TrajectoryPoint<QuadrotorModel>{.time_s = 1.0,
                                      .state = create_identity_state(),
                                      .control = Control::Ones()};

  EXPECT_FALSE(pt0 == pt1);
}

}  // namespace src