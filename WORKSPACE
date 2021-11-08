workspace(
    name = "lie_dynamics",
)

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository", "new_git_repository")

### External Dependencies ###

## Gtest ##
git_repository(
    name = "gtest",
    branch = "v1.10.x",
    remote = "https://github.com/google/googletest",
)

## LLVM toolchain ##
BAZEL_TOOLCHAIN_TAG = "0.6.3"

BAZEL_TOOLCHAIN_SHA = "da607faed78c4cb5a5637ef74a36fdd2286f85ca5192222c4664efec2d529bb8"

http_archive(
    name = "com_grail_bazel_toolchain",
    canonical_id = BAZEL_TOOLCHAIN_TAG,
    sha256 = BAZEL_TOOLCHAIN_SHA,
    strip_prefix = "bazel-toolchain-{tag}".format(tag = BAZEL_TOOLCHAIN_TAG),
    url = "https://github.com/grailbio/bazel-toolchain/archive/{tag}.tar.gz".format(tag = BAZEL_TOOLCHAIN_TAG),
)

load("@com_grail_bazel_toolchain//toolchain:deps.bzl", "bazel_toolchain_dependencies")

bazel_toolchain_dependencies()

load("@com_grail_bazel_toolchain//toolchain:rules.bzl", "llvm_toolchain")

llvm_toolchain(
    name = "llvm_toolchain",
    llvm_version = "13.0.0",
)

load("@llvm_toolchain//:toolchains.bzl", "llvm_register_toolchains")

llvm_register_toolchains()

## Rules Foreign ##
http_archive(
    name = "rules_foreign_cc",
    sha256 = "69023642d5781c68911beda769f91fcbc8ca48711db935a75da7f6536b65047f",
    strip_prefix = "rules_foreign_cc-0.6.0",
    url = "https://github.com/bazelbuild/rules_foreign_cc/archive/0.6.0.tar.gz",
)

load("@rules_foreign_cc//foreign_cc:repositories.bzl", "rules_foreign_cc_dependencies")

# This sets up some common toolchains for building targets. For more details, please see
# https://bazelbuild.github.io/rules_foreign_cc/0.6.0/flatten.html#rules_foreign_cc_dependencies
rules_foreign_cc_dependencies()

## Eigen ##
http_archive(
    name = "eigen",
    build_file = "@//:third_party/eigen.BUILD",
    sha256 = "0b73c17efcc4cbe6e689313a950f75281d82d2083683b88330e262dead3f5ce3",
    url = "https://eigen.googlesource.com/mirror/+archive/refs/heads/3.4.tar.gz",
)

## Micro Lie ##
new_git_repository(
    name = "com_github_artivis_manif",
    build_file = "@//:third_party/manif.BUILD",
    commit = "ab560a3a1dac3f0f6bf5154056bb4408f4c9f67c",
    remote = "https://github.com/artivis/manif",
    shallow_since = "1632662098 -0400",
)
