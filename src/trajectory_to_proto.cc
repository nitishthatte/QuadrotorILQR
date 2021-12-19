
#include "src/trajectory_to_proto.hh"

#include <algorithm>

#include "src/trajectory.pb.h"

namespace src {

proto::SE3 to_proto(const manif::SE3d &transform) {
  proto::SE3 proto_transform{};
  proto_transform.mutable_rot()->set_w(transform.quat().coeffs()[3]);
  proto_transform.mutable_rot()->set_x(transform.quat().coeffs()[0]);
  proto_transform.mutable_rot()->set_y(transform.quat().coeffs()[1]);
  proto_transform.mutable_rot()->set_z(transform.quat().coeffs()[2]);

  proto_transform.mutable_translation()->set_c0(transform.translation()[0]);
  proto_transform.mutable_translation()->set_c1(transform.translation()[1]);
  proto_transform.mutable_translation()->set_c2(transform.translation()[2]);

  return proto_transform;
}

manif::SE3d from_proto(const proto::SE3 &proto_transform) {
  Eigen::Vector3d translation{{proto_transform.translation().c0(),
                               proto_transform.translation().c1(),
                               proto_transform.translation().c2()}};
  Eigen::Quaterniond quat{proto_transform.rot().w(), proto_transform.rot().x(),
                          proto_transform.rot().y(), proto_transform.rot().z()};
  return manif::SE3d{translation, quat};
}

proto::LieTrajectoryPoint to_proto(const TrajectoryPoint<LieDynamics> &pt) {
  proto::LieTrajectoryPoint proto_pt{};
  proto_pt.set_time_s(pt.time_s);
  *proto_pt.mutable_state() = to_proto(pt.state);
  *proto_pt.mutable_control() = to_proto(pt.control);
  return proto_pt;
}
TrajectoryPoint<LieDynamics> from_proto(
    const proto::LieTrajectoryPoint &proto_pt) {
  return {.time_s = proto_pt.time_s(),
          .state = from_proto(proto_pt.state()),
          .control = from_proto(proto_pt.control())};
}

proto::LieTrajectory to_proto(const Trajectory<LieDynamics> &traj) {
  proto::LieTrajectory proto_traj;
  for (const auto &pt : traj) {
    auto pt_ptr = proto_traj.add_points();
    *pt_ptr = to_proto(pt);
  }
  return proto_traj;
}

Trajectory<LieDynamics> from_proto(const proto::LieTrajectory &proto_traj) {
  Trajectory<LieDynamics> traj;
  std::transform(proto_traj.points().cbegin(), proto_traj.points().cend(),
                 std::back_inserter(traj),
                 [](const auto &proto_pt) { return from_proto(proto_pt); });
  return traj;
}

}  // namespace src