#pragma once

#include <memory>

#include "cyber/cyber.h"

#include "modules/drivers/proto/pointcloud.pb.h"
#include "cyber/component/component.h"

#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/segmentation/sac_segmentation.h>
#include "cyber/common/file.h"
#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"
#include "cyber/base/concurrent_object_pool.h"


using apollo::drivers::PointCloud;
using apollo::cyber::Writer;
using apollo::cyber::base::CCObjectPool;

namespace apollo {
    namespace road_detect {
        class RoadDetectComponent : public cyber::Component<apollo::drivers::PointCloud> {
        public:
            RoadDetectComponent() = default;

            ~RoadDetectComponent() = default;

            bool Init() override;

            bool Proc(const std::shared_ptr<apollo::drivers::PointCloud> &msg) override;

            pcl::PointCloud<pcl::PointXYZ>::Ptr ConvertToPCL(std::shared_ptr<apollo::drivers::PointCloud> msg,
                    float roi_x_min = -1000, float roi_x_max = 1000, float roi_y_min = -1000, float roi_y_max = 1000,
                    float roi_z_min = -1000, float roi_z_max = 1000);

        private:
            std::shared_ptr<Writer<PointCloud>> writer = nullptr;
            std::shared_ptr<CCObjectPool<PointCloud>> road_pool = nullptr;
            int seq = 0;
        };

        CYBER_REGISTER_COMPONENT(RoadDetectComponent);
    }
}