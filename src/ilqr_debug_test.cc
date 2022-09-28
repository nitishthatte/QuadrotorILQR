#include "ilqr_debug.hh"

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

ILQRIterDebug<QuadrotorModel> create_debug() {
  return ILQRIterDebug<QuadrotorModel>{
      .iter = 55,
      .trajectory = {TrajectoryPoint<QuadrotorModel>{
          .time_s = 0.0,
          .state = create_identity_state(),
          .control = Control::Zero()}},
      .cost = 23.3};
}

}  // namespace

TEST(ILQRIterDebug, EqualityOperatorReturnsTrueIfDebugSame) {
  const auto debug = create_debug();
  EXPECT_EQ(debug, debug);
}

TEST(ILQRIterDebug, EqualityOperatorReturnsFalseIfIterDifferent) {
  const auto debug_0 = create_debug();
  auto debug_1 = debug_0;
  debug_1.iter = 10;

  EXPECT_NE(debug_0, debug_1);
}

TEST(ILQRIterDebug, EqualityOperatorReturnsFalseIfTrajDifferent) {
  const auto debug_0 = create_debug();
  auto debug_1 = debug_0;
  debug_1.trajectory.front().time_s = 10.0;

  EXPECT_NE(debug_0, debug_1);
}

TEST(ILQRIterDebug, EqualityOperatorReturnsFalseIfCostDifferent) {
  const auto debug_0 = create_debug();
  auto debug_1 = debug_0;
  debug_1.cost = 10.0;

  EXPECT_NE(debug_0, debug_1);
}
}  // namespace src