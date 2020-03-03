#include <iostream>
#include "cyber/cyber.h"
#include "road_detect_component.h"
#include "modules/drivers/proto/pointcloud.pb.h"



RoadDetectComponent detector = RoadDetectComponent();

void MessageCallback(const std::shared_ptr<apollo::drivers::PointCloud> &msg) {
    //std::cout << msg->point(0).x() << std::endl;
    detector.Proc(msg);
    auto talker = detector.detect_node->CreateWriter<apollo::drivers::PointCloud>("/road_points");
    talker->Write(msg);
}

int main(int argc, char *argv[]) {
    std::cout << "Start" << std::endl;
    detector.Init(argv);
    auto listener =
            detector.detect_node->CreateReader<apollo::drivers::PointCloud>(
                    "/apollo/sensor/lidar128/compensator/PointCloud2", MessageCallback);
    apollo::cyber::WaitForShutdown();
    return 0;
}

