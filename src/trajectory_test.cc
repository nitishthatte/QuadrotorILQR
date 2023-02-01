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

TrajectoryPoint<QuadrotorModel> create_trajectory_point() {
  return TrajectoryPoint<QuadrotorModel>{.time_s = 0.0,
                                         .state = create_identity_state(),
                                         .control = Control::Zero()};
}
}  // namespace

TEST(TrajectoryPoint, EqualityOperatorReturnsTrueIfTrajectoryPointsSame) {
  const auto pt = create_trajectory_point();
  EXPECT_EQ(pt, pt);
}

TEST(TrajectoryPoint,
     EqualityOperatorReturnsFalseIfTrajectoryPointsTimeDifferent) {
  const auto pt0 = create_trajectory_point();
  auto pt1 = pt0;
  pt1.time_s = 1.0;

  EXPECT_NE(pt0, pt1);
}

TEST(TrajectoryPoint,
     EqualityOperatorReturnsFalseIfTrajectoryPointsStateDifferent) {
  const auto pt0 = create_trajectory_point();

  auto pt1 = pt0;
  manif::SE3Tangentd state_1_offset{};
  state_1_offset.coeffs()(2) = 1.0;
  pt1.state.inertial_from_body += state_1_offset;

  EXPECT_NE(pt0, pt1);
}

TEST(TrajectoryPoint,
     EqualityOperatorReturnsFalseIfTrajectoryPointsControlDifferent) {
  const auto pt0 = create_trajectory_point();
  auto pt1 = create_trajectory_point();
  pt1.control = Control::Ones();

  EXPECT_NE(pt0, pt1);
}

}  // namespace src