
#pragma once

#include "src/ilqr_debug.hh"
#include "src/ilqr_debug.pb.h"
#include "src/quadrotor_model.hh"

namespace src::proto {

proto::QuadrotorILQRIterDebug to_proto(
    const ILQRIterDebug<QuadrotorModel> &debug);
ILQRIterDebug<QuadrotorModel> from_proto(
    const proto::QuadrotorILQRIterDebug &proto_debug);

proto::QuadrotorILQRDebug to_proto(const ILQRDebug<QuadrotorModel> &debug);
ILQRDebug<QuadrotorModel> from_proto(
    const proto::QuadrotorILQRDebug &proto_debug);

}  // namespace src::proto