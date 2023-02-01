load("@rules_foreign_cc//foreign_cc:cmake.bzl", "cmake")

filegroup(
    name = "all_srcs",
    srcs = glob(["**"]),
    visibility = ["//visibility:private"],
)

cmake(
    name = "wise_enum",
    lib_source = ":all_srcs",
    out_headers_only = True,
    visibility = ["//visibility:public"],
)
