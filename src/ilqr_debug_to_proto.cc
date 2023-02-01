
#include "src/ilqr_debug_to_proto.hh"

#include "src/trajectory_to_proto.hh"

namespace src::proto {

proto::QuadrotorILQRIterDebug to_proto(
    const ILQRIterDebug<QuadrotorModel> &debug) {
  proto::QuadrotorILQRIterDebug proto_debug;
  *proto_debug.mutable_trajectory() = to_proto(debug.trajectory);
  proto_debug.set_cost(debug.cost);

  return proto_debug;
}

ILQRIterDebug<QuadrotorModel> from_proto(
    const proto::QuadrotorILQRIterDebug &proto_debug) {
  return ILQRIterDebug<QuadrotorModel>{
      .trajectory = from_proto(proto_debug.trajectory()),
      .cost = proto_debug.cost()};
}

proto::QuadrotorILQRDebug to_proto(const ILQRDebug<QuadrotorModel> &debug) {
  proto::QuadrotorILQRDebug proto_debug;
  for (const auto &iter : debug) {
    auto iter_ptr = proto_debug.add_iter_debugs();
    *iter_ptr = to_proto(iter);
  }
  return proto_debug;
}

ILQRDebug<QuadrotorModel> from_proto(
    const proto::QuadrotorILQRDebug &proto_debug) {
  ILQRDebug<QuadrotorModel> debug;
  debug.reserve(proto_debug.iter_debugs_size());
  for (const auto &proto_iter_debug : proto_debug.iter_debugs()) {
    debug.emplace_back(from_proto(proto_iter_debug));
  }
  return debug;
}
}  // namespace src::proto