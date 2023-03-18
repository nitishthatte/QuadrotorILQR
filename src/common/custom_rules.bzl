"""Custom Bazel Rules"""

load("@com_google_protobuf//:protobuf.bzl", "py_proto_library")
load("@py_deps//:requirements.bzl", "requirement")
load("@rules_cc//cc:defs.bzl", "cc_proto_library")
load("@rules_proto//proto:defs.bzl", "proto_library")
load("@rules_python//python:defs.bzl", "py_binary")

def proto_lib_collection(name, srcs, deps = [], visibility = None):
    """Generates a proto libraries for the given proto sources.

    Creates a proto_library, cc_proto_library, and py_proto_library.  The output
    target names will be <name>_proto, <name_prefix>_cc_proto,
    <name_prefix>_py_proto respectively.

    Args:
      name: The proto library name. Must end with "_proto". The cc and py
        library will automatically end with _cc_proto and _py_proto instead.
      srcs: The source .proto files.
      deps: The dependent .proto files.
      visibility: The visiblity.
    """
    if name[-6:] != "_proto":
        fail("the target name {} does not end with _proto!".format(name))

    proto_library(
        name = name,
        srcs = srcs,
        deps = deps,
        visibility = visibility,
    )
    cc_proto_name = name[0:-6] + "_cc_proto"
    cc_proto_library(
        name = cc_proto_name,
        deps = [":" + name],
        visibility = visibility,
    )

    py_proto_name = name[0:-6] + "_py_proto"
    py_deps = [dep[0:-6] + "_py_proto" for dep in deps]
    py_proto_library(
        name = py_proto_name,
        srcs = srcs,
        deps = py_deps + ["@com_google_protobuf//:protobuf_python"],
        visibility = visibility,
    )

def irepl(name, data = [], deps = []):
    py_binary(
        name = name,
        srcs = ["//src/common:irepl.py"],
        main = "//src/common:irepl.py",
        data = data,
        deps = [
            requirement("ipython"),
        ] + deps,
    )
