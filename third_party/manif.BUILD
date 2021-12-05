load("@rules_foreign_cc//foreign_cc:cmake.bzl", "cmake")

filegroup(
    name = "all_srcs",
    srcs = glob(["**"]),
    visibility = ["//visibility:private"],
)

cmake(
    name = "manif",
    cache_entries = {
        "EIGEN3_INCLUDE_DIR": "$EXT_BUILD_DEPS/include",
    },
    lib_source = ":all_srcs",
    out_headers_only = True,
    visibility = ["//visibility:public"],
    deps = ["@eigen"],
)
