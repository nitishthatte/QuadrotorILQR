#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

#include "common/pybind_cast.hh"
#include "src/ilqr.hh"
#include "src/ilqr_options.pb.h"
#include "src/ilqr_options_to_proto.hh"
#include "src/trajectory.pb.h"
#include "src/trajectory_to_proto.hh"

namespace py = pybind11;

PYBIND11_PROTO_CASTER(src::proto::LieTrajectory, "LieTrajectory",
                      "src.trajectory_pb2");
PYBIND11_PROTO_CASTER(src::proto::ILQROptions, "ILQROptions",
                      "src.ilqr_options_pb2");

using LieCostFunction = src::CostFunction<src::LieDynamics>;

namespace src {
ILQR<LieDynamics> init(const LieCostFunction::CostHessianStateState &Q,
                       const LieCostFunction::CostHessianControlControl &R,
                       const proto::LieTrajectory &desired_traj,
                       const proto::ILQROptions &options) {
  CostFunction<LieDynamics> cost_func{Q, R, from_proto(desired_traj)};
  return ILQR<LieDynamics>{std::move(cost_func), from_proto(options)};
}

proto::LieTrajectory solve(const src::ILQR<src::LieDynamics> &self,
                           const proto::LieTrajectory &initial_traj) {
  return to_proto(self.solve(from_proto(initial_traj)));
}
}  // namespace src

PYBIND11_MODULE(lie_ilqr_binding, m) {
  py::class_<src::ILQR<src::LieDynamics>>(m, "LieILQR")
      .def(py::init(&src::init))
      .def("solve", &src::solve);
}