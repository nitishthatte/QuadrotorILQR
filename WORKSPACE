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
