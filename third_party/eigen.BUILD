# Description:
#   Eigen is a C++ template library for linear algebra: vectors,
#   matrices, and related algorithms.

load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "eigen",
    hdrs = glob([
        "Eigen/**",
        "unsupported/**",
    ]),
    includes = ["."],
    visibility = ["//visibility:public"],
)
