#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

#include "common/pybind_cast.hh"
#include "pybind11/eigen.h"
#include "quadrotor_model.hh"
#include "src/ilqr.hh"
#include "src/ilqr_options.pb.h"
#include "src/ilqr_options_to_proto.hh"
#include "src/trajectory.pb.h"
#include "src/trajectory_to_proto.hh"

namespace py = pybind11;

PYBIND11_PROTO_CASTER(src::proto::QuadrotorTrajectory, "QuadrotorTrajectory",
                      "src.trajectory_pb2");
PYBIND11_PROTO_CASTER(src::proto::ILQROptions, "ILQROptions",
                      "src.ilqr_options_pb2");

namespace src {
using QuadrotorCostFunction = CostFunction<src::QuadrotorModel>;

ILQR<QuadrotorModel> init(
    const double mass_kg, Eigen::Matrix3d inertia, const double arm_length_m,
    const double torque_to_thrust_ratio_m, const double g_mpss,
    const QuadrotorCostFunction::CostHessianStateState &Q,
    const QuadrotorCostFunction::CostHessianControlControl &R,
    const proto::QuadrotorTrajectory &desired_traj, const double dt_s,
    const proto::ILQROptions &options) {
  QuadrotorModel model{mass_kg, inertia, arm_length_m, torque_to_thrust_ratio_m,
                       g_mpss};
  QuadrotorCostFunction cost_func{Q, R, from_proto(desired_traj)};
  return ILQR<QuadrotorModel>{std::move(model), std::move(cost_func), dt_s,
                              from_proto(options)};
}

proto::QuadrotorTrajectory solve(
    const src::ILQR<src::QuadrotorModel> &self,
    const proto::QuadrotorTrajectory &initial_traj) {
  return src::proto::to_proto(self.solve(from_proto(initial_traj)));
}
}  // namespace src

PYBIND11_MODULE(quadrotor_ilqr_binding, m) {
  py::class_<src::ILQR<src::QuadrotorModel>>(m, "QuadrotorILQR")
      .def(py::init(&src::init))
      .def("solve", &src::solve);
}