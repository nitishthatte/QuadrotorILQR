#include "ilqr_debug_to_proto.hh"

#include <gtest/gtest.h>

#include "src/quadrotor_model.hh"
#include "src/trajectory_to_proto.hh"

namespace src::proto {

using State = QuadrotorModel::State;
using Control = QuadrotorModel::Control;

namespace {
State create_identity_state() {
  return State{.inertial_from_body = manif::SE3d::Identity(),
               .body_velocity = manif::SE3Tangentd::Zero()};
}

ILQRIterDebug<QuadrotorModel> create_debug() {
  return ILQRIterDebug<QuadrotorModel>{
      .trajectory = {TrajectoryPoint<QuadrotorModel>{
          .time_s = 0.0,
          .state = create_identity_state(),
          .control = Control::Zero()}},
      .cost = 23.3};
}

}  // namespace

TEST(QuadrotorILQRDebug, RoundTripTest) {
  const auto d0 = create_debug();
  auto d1 = create_debug();
  d1.trajectory.front().time_s = 1.0;
  d1.cost = 5;

  ILQRDebug<QuadrotorModel> debug{d0, d1};

  const auto round_trip = from_proto(to_proto(debug));

  EXPECT_EQ(debug, round_trip);
}
}  // namespace src::proto