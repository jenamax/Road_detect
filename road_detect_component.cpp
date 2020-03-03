#include "road_detect_component.h"
#include <iostream>

#include "cyber/common/file.h"

bool RoadDetectComponent::Init(char *argv[]) {
    apollo::cyber::Init(argv[0]);
    this->detect_node = apollo::cyber::CreateNode("road_detect");
    return apollo::cyber::OK();
}

bool RoadDetectComponent::Proc(const std::shared_ptr<apollo::drivers::PointCloud>& msg) {
    return apollo::cyber::OK();
}





