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
    hdrs = ["road_detect_component.h", "DigitalFilters-master/DigitalFilters.h", "DigitalFilters-master/CircularDelay/CircularDelay.hpp"],
    deps = [
        "//cyber",
        "//modules/drivers/proto:sensor_proto",
        "//modules/localization/proto:localization_proto",
        "//modules/localization/proto:gps_proto",
        "//modules/common/proto:geometry_proto",
        "//modules/map/hdmap:hdmap",
        "//modules/map/hdmap:hdmap_util",
        "@pcl",
        "@eigen",
        "@com_google_absl//absl/strings",
    ],
)
