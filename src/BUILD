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
