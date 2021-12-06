load("@rules_python//python:defs.bzl", "py_binary", "py_runtime", "py_runtime_pair")
load("@pybind11_bazel//:build_defs.bzl", "pybind_extension")
load("@py_deps//:requirements.bzl", "requirement")

py_runtime(
    name = "python3_runtime",
    files = ["@python_interpreter//:files"],
    interpreter = "@python_interpreter//:python_bin",
    python_version = "PY3",
    visibility = ["//visibility:public"],
)

py_runtime_pair(
    name = "py_runtime_pair",
    py2_runtime = None,
    py3_runtime = ":python3_runtime",
)

toolchain(
    name = "py_3_toolchain",
    toolchain = ":py_runtime_pair",
    toolchain_type = "@bazel_tools//tools/python:toolchain_type",
)

pybind_extension(
    name = "pybind_example",
    srcs = ["pybind_example.cc"],
)

py_binary(
    name = "irepl",
    srcs = ["irepl.py"],
    data = ["pybind_example.so"],
    deps = [
        requirement("ipython"),
    ],
)
