#pragma once

#include <memory>

#include "cyber/cyber.h"

#include "modules/drivers/proto/pointcloud.pb.h"
#include "cyber/component/component.h"



class RoadDetectComponent {
public:
    RoadDetectComponent() = default;
    ~RoadDetectComponent() = default;
    std::shared_ptr<apollo::cyber::Node> detect_node;

    bool Init(char *argv[]);

    bool Proc(const std::shared_ptr<apollo::drivers::PointCloud>& msg);
private:
};

//CYBER_REGISTER_COMPONENT(RoadDetectComponent);

