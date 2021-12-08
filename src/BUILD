load("@rules_cc//cc:defs.bzl", "cc_library")
load("//src/common:custom_rules.bzl", "proto_lib_collection")

cc_library(
    name = "dynamics",
    srcs = ["dynamics.cc"],
    hdrs = ["dynamics.hh"],
    deps = [
        "@com_github_artivis_manif//:manif",
    ],
)

cc_test(
    name = "dynamics_test",
    size = "small",
    srcs = ["dynamics_test.cc"],
    deps = [
        ":dynamics",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "cost",
    hdrs = ["cost.hh"],
    deps = [
        ":dynamics",
        "@com_github_artivis_manif//:manif",
    ],
)

cc_test(
    name = "cost_test",
    size = "small",
    srcs = ["cost_test.cc"],
    deps = [
        ":cost",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "trajectory",
    hdrs = ["trajectory.hh"],
    deps = [
        ":dynamics",
    ],
)

cc_test(
    name = "trajectory_test",
    size = "small",
    srcs = ["trajectory_test.cc"],
    deps = [
        ":trajectory",
        "@gtest//:gtest_main",
    ],
)

proto_lib_collection(
    name = "trajectory_proto",
    srcs = ["trajectory.proto"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "trajectory_to_proto",
    srcs = ["trajectory_to_proto.cc"],
    hdrs = ["trajectory_to_proto.hh"],
    deps = [
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
    name = "ilqr",
    hdrs = ["ilqr.hh"],
    deps = [
        ":cost",
        ":dynamics",
        ":trajectory",
    ],
)

cc_test(
    name = "ilqr_test",
    size = "small",
    srcs = ["ilqr_test.cc"],
    deps = [
        ":ilqr",
        "@gtest//:gtest_main",
    ],
)
