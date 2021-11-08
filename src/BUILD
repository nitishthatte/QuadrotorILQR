load("@rules_cc//cc:defs.bzl", "cc_library")

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
    srcs = ["cost.cc"],
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
