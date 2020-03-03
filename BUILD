cc_library(
    name = "road_detect_lib",
    srcs = ["road_detect_component.cpp",],
    hdrs = ["road_detect_component.h",],
    deps = [
        "//cyber",
        "//modules/drivers/proto:sensor_proto",
    ],
)

cc_binary(
    name = "road_detect",
     srcs = ["lidar_road_detect.cpp",],
     deps = [
        "//cyber",
        "//modules/drivers/proto:sensor_proto",
        ":road_detect_lib",
     ],
)
