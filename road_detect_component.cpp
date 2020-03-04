#include "road_detect_component.h"
#include <iostream>

#include "cyber/common/file.h"
#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"


namespace apollo {
    namespace road_detect {
        bool RoadDetectComponent::Init() {
            writer = detect_node->CreateWriter<PointCloud>("road_points");
            return apollo::cyber::OK();
        }

        bool RoadDetectComponent::Proc(const std::shared_ptr<PointCloud> &msg) {
           // writer->Write(msg);
            return apollo::cyber::OK();
        }
    }
}



