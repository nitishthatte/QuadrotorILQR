
#pragma once

#include "src/trajectory.hh"
#include "src/trajectory.pb.h"

namespace src {

proto::SE3 to_proto(const manif::SE3d &transform);
manif::SE3d from_proto(const proto::SE3 &proto_transform);

proto::LieTrajectoryPoint to_proto(const TrajectoryPoint<LieDynamics> &pt);
TrajectoryPoint<LieDynamics> from_proto(
    const proto::LieTrajectoryPoint &proto_pt);

proto::LieTrajectory to_proto(const Trajectory<LieDynamics> &traj);
Trajectory<LieDynamics> from_proto(const proto::LieTrajectory &proto_traj);

}  // namespace src