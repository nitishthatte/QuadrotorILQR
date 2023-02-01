workspace(
    name = "lie_dynamics",
)

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository", "new_git_repository")

### External Dependencies ###

## Eigen ##
http_archive(
    name = "eigen",
    build_file = "@//:third_party/eigen.BUILD",
    sha256 = "8586084f71f9bde545ee7fa6d00288b264a2b7ac3607b974e54d13e7162c1c72",
    strip_prefix = "eigen-3.4.0",
    url = "https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz",
)

## Gtest ##
git_repository(
    name = "gtest",
    commit = "703bd9caab50b139428cea1aaff9974ebee5742e",
    remote = "https://github.com/google/googletest",
    shallow_since = "1570114335 -0400",
)

## Micro Lie ##
new_git_repository(
    name = "com_github_artivis_manif",
    build_file = "@//:third_party/manif.BUILD",
    commit = "ab560a3a1dac3f0f6bf5154056bb4408f4c9f67c",
    remote = "https://github.com/artivis/manif",
    shallow_since = "1632662098 -0400",
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

http_archive(
    name = "rules_python",
    sha256 = "cd6730ed53a002c56ce4e2f396ba3b3be262fd7cb68339f0377a45e8227fe332",
    url = "https://github.com/bazelbuild/rules_python/releases/download/0.5.0/rules_python-0.5.0.tar.gz",
)

## Python Interpreter ##
_configure_python_based_on_os = """
if [[ "$OSTYPE" == "darwin"* ]]; then
    ./configure --prefix=$(pwd)/bazel_install --with-openssl=$(brew --prefix openssl)
else
    ./configure --prefix=$(pwd)/bazel_install
fi
"""

http_archive(
    name = "python_interpreter",
    build_file_content = """
exports_files(["python_bin"])
filegroup(
    name = "files",
    srcs = glob(["bazel_install/**"], exclude = ["**/* *"]),
    visibility = ["//visibility:public"],
)
""",
    patch_cmds = [
        "mkdir $(pwd)/bazel_install",
        _configure_python_based_on_os,
        "make",
        "make install",
        "ln -s bazel_install/bin/python3 python_bin",
    ],
    sha256 = "6eed8415b7516fb2f260906db5d48dd4c06acc0cb24a7d6cc15296a604dcdc48",
    strip_prefix = "Python-3.10.7",
    urls = ["https://www.python.org/ftp/python/3.10.7/Python-3.10.7.tar.xz"],
)

register_toolchains("//:py_3_toolchain")

# Create a central external repo, @my_deps, that contains Bazel targets for all the
# third-party packages specified in the requirements.txt file.
load("@rules_python//python:pip.bzl", "pip_parse")

pip_parse(
    name = "py_deps",
    python_interpreter_target = "@python_interpreter//:python_bin",
    requirements_lock = "//:requirements.txt",
)

# Load the starlark macro which will define your dependencies.
load("@py_deps//:requirements.bzl", "install_deps")

# Call it to define repos for your requirements.
install_deps()

## Pybind ##
git_repository(
    name = "pybind11_bazel",
    commit = "72cbbf1fbc830e487e3012862b7b720001b70672",
    remote = "https://github.com/pybind/pybind11_bazel",
    shallow_since = "1638580149 -0800",
)

http_archive(
    name = "pybind11",
    build_file = "@pybind11_bazel//:pybind11.BUILD",
    sha256 = "eacf582fa8f696227988d08cfc46121770823839fe9e301a20fbce67e7cd70ec",
    strip_prefix = "pybind11-2.10.0",
    urls = ["https://github.com/pybind/pybind11/archive/refs/tags/v2.10.0.tar.gz"],
)

load("@pybind11_bazel//:python_configure.bzl", "python_configure")

python_configure(
    name = "local_config_python",
)

## Protobuf ##
http_archive(
    name = "rules_proto",
    sha256 = "66bfdf8782796239d3875d37e7de19b1d94301e8972b3cbd2446b332429b4df1",
    strip_prefix = "rules_proto-4.0.0",
    urls = [
        "https://mirror.bazel.build/github.com/bazelbuild/rules_proto/archive/refs/tags/4.0.0.tar.gz",
        "https://github.com/bazelbuild/rules_proto/archive/refs/tags/4.0.0.tar.gz",
    ],
)

load("@rules_proto//proto:repositories.bzl", "rules_proto_dependencies", "rules_proto_toolchains")

rules_proto_dependencies()

rules_proto_toolchains()

http_archive(
    name = "com_google_protobuf",
    sha256 = "80631d5a18d51daa3a1336e340001ad4937e926762f21144c62d26fe2a8d71fe",
    strip_prefix = "protobuf-3.19.1",
    urls = [
        "https://github.com/protocolbuffers/protobuf/releases/download/v3.19.1/protobuf-all-3.19.1.tar.gz",
    ],
)

git_repository(
    name = "pybind11_protobuf",
    commit = "158cc03371a5bee6e771715167a1b02776b993a7",
    remote = "https://github.com/pybind/pybind11_protobuf",
    shallow_since = "1661450640 -0700",
)

## ABSL ##
http_archive(
    name = "com_google_absl",
    sha256 = "91ac87d30cc6d79f9ab974c51874a704de9c2647c40f6932597329a282217ba8",
    strip_prefix = "abseil-cpp-20220623.1",
    urls = ["https://github.com/abseil/abseil-cpp/archive/refs/tags/20220623.1.tar.gz"],
)

## Wise Enum ##
new_git_repository(
    name = "wise_enum",
    build_file = "@//:third_party/wise_enum.BUILD",
    commit = "34ac79f7ea2658a148359ce82508cc9301e31dd3",
    remote = "https://github.com/quicknir/wise_enum",
)

# Needed by Protobuf
bind(
    name = "python_headers",
    actual = "@local_config_python//:python_headers",
)
