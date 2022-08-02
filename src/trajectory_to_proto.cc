
#include "src/trajectory_to_proto.hh"

#include <algorithm>

#include "quadrotor_model.hh"
#include "src/trajectory.pb.h"

namespace src::proto {

proto::Vec3 to_proto(const Eigen::Vector3d &vec) {
  proto::Vec3 proto_vec{};
  proto_vec.set_c0(vec[0]);
  proto_vec.set_c1(vec[1]);
  proto_vec.set_c2(vec[2]);
  return proto_vec;
}

Eigen::Vector3d from_proto(const proto::Vec3 &proto_vec) {
  Eigen::Vector3d vec{};
  vec[0] = proto_vec.c0();
  vec[1] = proto_vec.c1();
  vec[2] = proto_vec.c2();
  return vec;
}

proto::Vec4 to_proto(const Eigen::Vector4d &vec) {
  proto::Vec4 proto_vec{};
  proto_vec.set_c0(vec[0]);
  proto_vec.set_c1(vec[1]);
  proto_vec.set_c2(vec[2]);
  proto_vec.set_c3(vec[3]);
  return proto_vec;
}

Eigen::Vector4d from_proto(const proto::Vec4 &proto_vec) {
  Eigen::Vector4d vec{};
  vec[0] = proto_vec.c0();
  vec[1] = proto_vec.c1();
  vec[2] = proto_vec.c2();
  vec[3] = proto_vec.c3();
  return vec;
}

proto::Vec6 to_proto(const Eigen::Vector<double, 6> &vec) {
  proto::Vec6 proto_vec{};
  proto_vec.set_c0(vec[0]);
  proto_vec.set_c1(vec[1]);
  proto_vec.set_c2(vec[2]);
  proto_vec.set_c3(vec[3]);
  proto_vec.set_c4(vec[4]);
  proto_vec.set_c5(vec[5]);
  return proto_vec;
}

Eigen::Vector<double, 6> from_proto(const proto::Vec6 &proto_vec) {
  Eigen::Vector<double, 6> vec{};
  vec[0] = proto_vec.c0();
  vec[1] = proto_vec.c1();
  vec[2] = proto_vec.c2();
  vec[3] = proto_vec.c3();
  vec[4] = proto_vec.c4();
  vec[5] = proto_vec.c5();
  return vec;
}

proto::SO3 to_proto(const manif::SO3d &rot) {
  proto::SO3 proto_rot{};
  proto_rot.mutable_quaternion()->set_c0(rot.w());
  proto_rot.mutable_quaternion()->set_c1(rot.x());
  proto_rot.mutable_quaternion()->set_c2(rot.y());
  proto_rot.mutable_quaternion()->set_c3(rot.z());
  return proto_rot;
}

manif::SO3d from_proto(const proto::SO3 &proto_rot) {
  return manif::SO3d{Eigen::Quaterniond{
      proto_rot.quaternion().c0(),
      proto_rot.quaternion().c1(),
      proto_rot.quaternion().c2(),
      proto_rot.quaternion().c3(),
  }};
}

proto::SE3 to_proto(const manif::SE3d &transform) {
  proto::SE3 proto_transform{};
  *proto_transform.mutable_translation() = to_proto(transform.translation());
  *proto_transform.mutable_rotation() =
      to_proto(manif::SO3d{transform.asSO3()});
  return proto_transform;
}

manif::SE3d from_proto(const proto::SE3 &proto_transform) {
  return manif::SE3d{from_proto(proto_transform.translation()),
                     from_proto(proto_transform.rotation())};
}

proto::QuadrotorState to_proto(const QuadrotorModel::State &state) {
  proto::QuadrotorState proto_state{};
  *proto_state.mutable_inertial_from_body() =
      to_proto(state.inertial_from_body);
  *proto_state.mutable_body_velocity() = to_proto(state.body_velocity.coeffs());
  return proto_state;
}

QuadrotorModel::State from_proto(const proto::QuadrotorState &proto_state) {
  return QuadrotorModel::State{
      .inertial_from_body = from_proto(proto_state.inertial_from_body()),
      .body_velocity = from_proto(proto_state.body_velocity())};
}

proto::QuadrotorTrajectoryPoint to_proto(
    const TrajectoryPoint<QuadrotorModel> &pt) {
  proto::QuadrotorTrajectoryPoint proto_pt{};
  proto_pt.set_time_s(pt.time_s);
  *proto_pt.mutable_state() = to_proto(pt.state);
  *proto_pt.mutable_control() = to_proto(pt.control);
  return proto_pt;
}
TrajectoryPoint<QuadrotorModel> from_proto(
    const proto::QuadrotorTrajectoryPoint &proto_pt) {
  return {.time_s = proto_pt.time_s(),
          .state = from_proto(proto_pt.state()),
          .control = from_proto(proto_pt.control())};
}

proto::QuadrotorTrajectory to_proto(const Trajectory<QuadrotorModel> &traj) {
  proto::QuadrotorTrajectory proto_traj;
  for (const auto &pt : traj) {
    auto pt_ptr = proto_traj.add_points();
    *pt_ptr = to_proto(pt);
  }
  return proto_traj;
}

Trajectory<QuadrotorModel> from_proto(
    const proto::QuadrotorTrajectory &proto_traj) {
  Trajectory<QuadrotorModel> traj;
  traj.reserve(proto_traj.points_size());
  for (const auto &proto_pt : proto_traj.points()) {
    traj.emplace_back(from_proto(proto_pt));
  }
  return traj;
}

}  // namespace src::proto