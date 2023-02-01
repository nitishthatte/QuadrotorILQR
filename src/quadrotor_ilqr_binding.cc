#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

#include "pybind11/eigen.h"
#include "pybind11_protobuf/native_proto_caster.h"
#include "quadrotor_model.hh"
#include "src/ilqr.hh"
#include "src/ilqr_debug.pb.h"
#include "src/ilqr_debug_to_proto.hh"
#include "src/ilqr_options.pb.h"
#include "src/ilqr_options_to_proto.hh"
#include "src/trajectory.pb.h"
#include "src/trajectory_to_proto.hh"

namespace py = pybind11;

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

std::pair<proto::QuadrotorTrajectory, proto::QuadrotorILQRDebug> solve(
    const src::ILQR<src::QuadrotorModel> &self,
    const proto::QuadrotorTrajectory &initial_traj) {
  const auto [opt_traj, debug] = self.solve(from_proto(initial_traj));
  auto proto_traj = src::proto::to_proto(opt_traj);
  auto proto_debug = src::proto::to_proto(debug);
  return std::make_pair(std::move(proto_traj), std::move(proto_debug));
}
}  // namespace src

PYBIND11_MODULE(quadrotor_ilqr_binding, m) {
  pybind11_protobuf::ImportNativeProtoCasters();

  py::class_<src::ILQR<src::QuadrotorModel>>(m, "QuadrotorILQR")
      .def(py::init(&src::init))
      .def("solve", &src::solve);
}