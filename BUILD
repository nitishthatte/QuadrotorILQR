load("@rules_python//python:defs.bzl", "py_binary", "py_runtime", "py_runtime_pair")
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

py_binary(
    name = "irepl",
    srcs = ["irepl.py"],
    data = [
        "//src:ilqr_binding.so",
    ],
    deps = [
        requirement("ipython"),
        "//src:trajectory_py_proto",
    ],
)
