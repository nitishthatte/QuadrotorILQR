#include <pybind11/pybind11.h>

#include "common/pybind_cast.hh"
#include "src/ilqr.hh"
#include "src/trajectory.pb.h"

namespace py = pybind11;

PYBIND11_PROTO_CASTER(src::proto::LieTrajectory, "LieTrajectory",
                      "src.trajectory_pb2");

/*
PYBIND11_MODULE(ilqr_binding, m) {
  m.doc() = "ilqr module";  // optional module docstring

  m.def("add", add, "A function that adds two quaternions");
}
PYBIND11_MODULE(lie_ilqr, m) {
  py::class_<src::ILQR<LieDynamics>>(m, "LieILQR")
      .def(py::init<const std::string &>())
      .def("setName", &Pet::setName)
      .def("getName", &Pet::getName);
}
*/