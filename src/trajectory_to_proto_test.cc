#include "trajectory_to_proto.hh"

#include <gtest/gtest.h>

#include "quadrotor_model.hh"

namespace src::proto {

using State = QuadrotorModel::State;
constexpr auto CONFIG_DIM = QuadrotorModel::CONFIG_DIM;
using Control = QuadrotorModel::Control;

TEST(QuadrotorTrajectory, RoundTripTest) {
  const State x0{.inertial_from_body =
                     manif::SE3Tangentd{Eigen::Vector<double, CONFIG_DIM>{
                                            1.0, 2.0, 3.0, 4.0, 5.0, 6.0}}
                         .exp(),
                 .body_velocity = Eigen::Vector<double, CONFIG_DIM>{
                     2.0, 3.0, 4.0, 5.0, 6.0, 7.0}};
  const Control u0 = Eigen::Vector4d{3.0, 4.0, 5.0, 6.0};

  const State x1{.inertial_from_body =
                     manif::SE3Tangentd{Eigen::Vector<double, CONFIG_DIM>{
                                            2.0, 3.0, 4.0, 5.0, 6.0, 7.0}}
                         .exp(),
                 .body_velocity = Eigen::Vector<double, CONFIG_DIM>{
                     3.0, 4.0, 5.0, 6.0, 7.0, 8.0}};
  const Control u1 = Eigen::Vector4d{4.0, 5.0, 6.0, 7.0};

  Trajectory<QuadrotorModel> traj{
      {.time_s = 1.0, .state = x0, .control = u0},
      {.time_s = 2.0, .state = x1, .control = u1},
  };

  const auto round_trip = from_proto(to_proto(traj));

  EXPECT_EQ(traj, round_trip);
}
}  // namespace src::proto