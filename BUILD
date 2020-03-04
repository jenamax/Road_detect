load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "road_detect_component.so",
    linkopts = ["-shared"],
    linkstatic = False,
    deps = [":road_detect_component_lib"],
)


cc_library(
    name = "road_detect_component_lib",
    srcs = ["road_detect_component.cpp",],
    hdrs = ["road_detect_component.h",],
    deps = [
        "//cyber",
        "//modules/drivers/proto:sensor_proto",
    ],
)
