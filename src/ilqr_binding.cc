#include <pybind11/pybind11.h>

#include "common/pybind_cast.hh"
#include "src/trajectory.pb.h"

namespace py = pybind11;

src::proto::Quaternion add(const src::proto::Quaternion &i,
                           const src::proto::Quaternion &j) {
  src::proto::Quaternion out;
  out.set_c0(i.c0() + j.c0());
  out.set_c1(i.c1() + j.c1());
  out.set_c2(i.c2() + j.c2());
  out.set_c3(i.c3() + j.c3());
  return out;
}

PYBIND11_CASTER(src::proto::Quaternion, "Quaternion", "src.trajectory_pb2");

// namespace pybind11 {
// namespace detail {
// template <>
// struct type_caster<src::proto::Quaternion> {
//  public:
//   PYBIND11_TYPE_CASTER(src::proto::Quaternion, _("Quaternion"));
//
//   bool load(handle src, bool) { /* Conversion part 1 (Python->C++): */
//     PyObject *source = src.ptr();
//     if (!PyObject_HasAttrString(source, "SerializeToString")) {
//       return false;
//     }
//     if (strcmp(Py_TYPE(source)->tp_name, "Quaternion")) {
//       return false;
//     }
//
//     PyObject *py_bytes_string =
//         PyObject_CallMethod(source, "SerializeToString", nullptr);
//     if (py_bytes_string == nullptr) {
//       return false;
//     }
//     assert(PyBytes_Check(py_bytes_string));
//     PyObject *py_bytes_array = PyByteArray_FromObject(
//         py_bytes_string); /*assert(PyBytesArray_Check(py_bytes_array));*/
//     const int len = PyByteArray_Size(py_bytes_array);
//     char *serialized_chars = PyByteArray_AsString(py_bytes_array);
//
//     const bool success =
//         value.ParseFromString(std::string(serialized_chars, len));
//     Py_DECREF(py_bytes_array);
//     Py_DECREF(py_bytes_string);
//     return success;
//   }
//
//   static handle cast(
//       const src::proto::Quaternion &src, return_value_policy /* policy */,
//       handle /* parent */) { /*Conversion part 2 (C++ -> Python)*/
//     std::string serialized;
//     src.SerializeToString(&serialized);
//     PyObject *bytes =
//         PyBytes_FromStringAndSize(serialized.data(), serialized.length());
//     assert(PyBytes_Check(bytes));
//     PyObject *module = PyImport_ImportModule("src.trajectory_pb2");
//     if (module == nullptr) {
//       throw std::runtime_error("Could not import src.trajectory_pb2");
//     }
//     PyObject *proto = PyObject_CallMethod(module, "Quaternion", nullptr);
//     if (module == nullptr) {
//       const auto str = "Unable to find Quaternion in src.trajectory_pb2";
//       throw std::runtime_error(str);
//     }
//     PyObject *method = PyUnicode_FromString(
//         "ParseFromString"); /*assert(PyUnicodeCheck(method));*/
//
//     PyObject *executed =
//         PyObject_CallMethodObjArgs(proto, method, bytes, nullptr);
//     if (executed == nullptr) {
//       throw std::runtime_error("Unable to parse Quaternion");
//     }
//     Py_DECREF(executed);
//     Py_DECREF(method);
//     Py_DECREF(module);
//     Py_DECREF(bytes);
//     return proto;
//   }
// };
// }  // namespace detail
// }  // namespace pybind11

PYBIND11_MODULE(ilqr_binding, m) {
  m.doc() = "ilqr module";  // optional module docstring

  m.def("add", add, "A function that adds two quaternions");
}