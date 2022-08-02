
#pragma once

#include "src/quadrotor_model.hh"
#include "src/trajectory.hh"
#include "src/trajectory.pb.h"

namespace src::proto {

proto::Vec3 to_proto(const Eigen::Vector3d &vec);
Eigen::Vector3d from_proto(const proto::Vec3 &proto_vec);

proto::Vec4 to_proto(const Eigen::Vector4d &vec);
Eigen::Vector4d from_proto(const proto::Vec4 &proto_vec);

proto::Vec6 to_proto(const Eigen::Vector<double, 6> &vec);
Eigen::Vector<double, 6> from_proto(const proto::Vec6 &proto_vec);

proto::SO3 to_proto(const manif::SO3d &rot);
manif::SO3d from_proto(const proto::SO3 &proto_rot);

proto::SE3 to_proto(const manif::SE3d &transform);
manif::SE3d from_proto(const proto::SE3 &proto_transform);

proto::QuadrotorState to_proto(const QuadrotorModel::State &state);
QuadrotorModel::State from_proto(const proto::QuadrotorState &proto_transform);

proto::QuadrotorTrajectoryPoint to_proto(
    const TrajectoryPoint<QuadrotorModel> &pt);
TrajectoryPoint<QuadrotorModel> from_proto(
    const proto::QuadrotorTrajectoryPoint &proto_pt);

proto::QuadrotorTrajectory to_proto(const Trajectory<QuadrotorModel> &traj);
Trajectory<QuadrotorModel> from_proto(
    const proto::QuadrotorTrajectory &proto_traj);

}  // namespace src::proto