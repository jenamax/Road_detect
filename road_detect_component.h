#pragma once

#include <memory>

#include "cyber/cyber.h"

#include "modules/drivers/proto/pointcloud.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/localization/proto/gps.pb.h"
#include "cyber/component/component.h"
#include "modules/common/proto/geometry.pb.h"

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
#include <pcl/kdtree/kdtree_flann.h>
#include "modules/map/hdmap/hdmap_util.h"



class Ptr;

using apollo::drivers::PointCloud;
using apollo::cyber::Writer;
using apollo::cyber::Reader;
using apollo::cyber::base::CCObjectPool;
using apollo::localization::Gps;
using apollo::localization::LocalizationEstimate;
using apollo::hdmap::HDMap;
using apollo::hdmap::RoadInfoConstPtr;

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

            pcl::PointIndices::Ptr cluster(pcl::PointXYZ searchPoint, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
            //void GnssCallback(const std::shared_ptr<apollo::localization::Gps> &gps_msg);

            const HDMap* map;
        private:
            std::shared_ptr<Writer<PointCloud>> writer = nullptr;
            std::shared_ptr<Reader<LocalizationEstimate>> gnss_listener = nullptr;
            std::shared_ptr<CCObjectPool<PointCloud>> road_pool = nullptr;
            int seq = 0;
        };

        CYBER_REGISTER_COMPONENT(RoadDetectComponent);
    }
}