load("@rules_cc//cc:defs.bzl", "cc_library")
load("//src/common:custom_rules.bzl", "irepl", "proto_lib_collection")
load("@pybind11_bazel//:build_defs.bzl", "pybind_extension")
load("@py_deps//:requirements.bzl", "requirement")

cc_library(
    name = "quadrotor_model",
    srcs = ["quadrotor_model.cc"],
    hdrs = ["quadrotor_model.hh"],
    deps = [
        "@com_github_artivis_manif//:manif",
    ],
)

cc_test(
    name = "quadrotor_model_test",
    size = "small",
    srcs = ["quadrotor_model_test.cc"],
    deps = [
        ":quadrotor_model",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "cost",
    hdrs = ["cost.hh"],
    deps = [
        ":trajectory",
        "@com_github_artivis_manif//:manif",
    ],
)

cc_test(
    name = "cost_test",
    size = "small",
    srcs = ["cost_test.cc"],
    deps = [
        ":cost",
        ":quadrotor_model",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "trajectory",
    hdrs = ["trajectory.hh"],
)

cc_test(
    name = "trajectory_test",
    size = "small",
    srcs = ["trajectory_test.cc"],
    deps = [
        ":quadrotor_model",
        ":trajectory",
        "@gtest//:gtest_main",
    ],
)

proto_lib_collection(
    name = "trajectory_proto",
    srcs = ["trajectory.proto"],
)

cc_library(
    name = "trajectory_to_proto",
    srcs = ["trajectory_to_proto.cc"],
    hdrs = ["trajectory_to_proto.hh"],
    deps = [
        ":quadrotor_model",
        ":trajectory",
        ":trajectory_cc_proto",
    ],
)

cc_test(
    name = "trajectory_to_proto_test",
    size = "small",
    srcs = ["trajectory_to_proto_test.cc"],
    deps = [
        ":trajectory_to_proto",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "ilqr_options",
    srcs = ["ilqr_options.cc"],
    hdrs = ["ilqr_options.hh"],
)

cc_test(
    name = "ilqr_options_test",
    size = "small",
    srcs = ["ilqr_options_test.cc"],
    deps = [
        ":ilqr_options",
        "@gtest//:gtest_main",
    ],
)

proto_lib_collection(
    name = "ilqr_options_proto",
    srcs = ["ilqr_options.proto"],
)

cc_library(
    name = "ilqr_options_to_proto",
    srcs = ["ilqr_options_to_proto.cc"],
    hdrs = ["ilqr_options_to_proto.hh"],
    deps = [
        ":ilqr_options",
        ":ilqr_options_cc_proto",
    ],
)

cc_test(
    name = "ilqr_options_to_proto_test",
    size = "small",
    srcs = ["ilqr_options_to_proto_test.cc"],
    deps = [
        ":ilqr_options_to_proto",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "ilqr",
    hdrs = ["ilqr.hh"],
    deps = [
        ":cost",
        ":ilqr_options",
        ":trajectory",
    ],
)

cc_test(
    name = "ilqr_test",
    size = "small",
    srcs = ["ilqr_test.cc"],
    deps = [
        ":ilqr",
        ":quadrotor_model",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "ilqr_debug",
    hdrs = ["ilqr_debug.hh"],
    deps = [
        ":trajectory",
    ],
)

cc_test(
    name = "ilqr_debug_test",
    size = "small",
    srcs = ["ilqr_debug_test.cc"],
    deps = [
        ":ilqr_debug",
        ":quadrotor_model",
        "@gtest//:gtest_main",
    ],
)

pybind_extension(
    name = "quadrotor_ilqr_binding",
    srcs = ["quadrotor_ilqr_binding.cc"],
    deps = [
        ":ilqr",
        ":ilqr_options_cc_proto",
        ":ilqr_options_to_proto",
        ":trajectory_cc_proto",
        ":trajectory_to_proto",
        "//src/common:pybind_cast",
    ],
)

irepl(
    name = "quadrotor_ilqr_irepl",
    data = [
        ":quadrotor_ilqr_binding.so",
    ],
    deps = [
        ":ilqr_options_py_proto",
        ":trajectory_py_proto",
        requirement("numpy"),
    ],
)

py_binary(
    name = "quadrotor_ilqr",
    srcs = ["quadrotor_ilqr.py"],
    data = [
        ":quadrotor_ilqr_binding.so",
        "//:quadrotor_stl_model",
    ],
    deps = [
        ":ilqr_options_py_proto",
        ":trajectory_py_proto",
        requirement("numpy"),
        requirement("scipy"),
        requirement("numpy-stl"),
        requirement("matplotlib"),
    ],
)
