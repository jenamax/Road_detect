#pragma once

#include <memory>

#include "cyber/cyber.h"

#include "modules/drivers/proto/pointcloud.pb.h"
#include "cyber/component/component.h"

using apollo::drivers::PointCloud;
using apollo::cyber::Writer;

namespace apollo {
    namespace road_detect {
        class RoadDetectComponent : public cyber::Component<apollo::drivers::PointCloud> {
        public:
            RoadDetectComponent() = default;

            ~RoadDetectComponent() = default;

            std::shared_ptr<apollo::cyber::Node> detect_node;

            bool Init() override;

            bool Proc(const std::shared_ptr<apollo::drivers::PointCloud> &msg) override;

        private:
            std::shared_ptr<Writer<PointCloud>> writer = nullptr;
        };

        CYBER_REGISTER_COMPONENT(RoadDetectComponent);
    }
}