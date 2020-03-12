#include "road_detect_component.h"
#include <iostream>



using std::cout;
using std::endl;
using pcl::PointXYZ;

namespace apollo {
    namespace road_detect {
        bool RoadDetectComponent::Init() {
            writer = node_->CreateWriter<PointCloud>("/apollo/road_pcl");
            return apollo::cyber::OK();
        }

        bool RoadDetectComponent::Proc(const std::shared_ptr<PointCloud> &msg) {
            auto start = cyber::Time::Now();
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            cloud = ConvertToPCL(msg);

            pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
            pcl::SACSegmentation<pcl::PointXYZ> seg;
            seg.setOptimizeCoefficients (true);
            seg.setModelType (pcl::SACMODEL_PLANE);
            seg.setMethodType (pcl::SAC_RANSAC);
            seg.setDistanceThreshold (0.2);
            seg.setInputCloud (cloud);
            seg.segment (*inliers, *coefficients);

            std::shared_ptr<PointCloud> msg_road ;
            msg_road = std::make_shared<PointCloud>();
            msg_road->mutable_point()->Reserve(240000);
            msg_road->Clear();
            msg_road->mutable_header()->set_timestamp_sec(msg->header().timestamp_sec());
            msg_road->mutable_header()->set_frame_id(msg->header().frame_id());
            msg_road->mutable_header()->set_lidar_timestamp(msg->header().lidar_timestamp());
            msg_road->set_measurement_time(msg->measurement_time());
            msg_road->set_height(1);
            msg_road->set_width(inliers->indices.size());
            msg_road->set_is_dense(msg->is_dense());
            for (unsigned int i = 0; i < inliers->indices.size(); i++){
                auto* point_new = msg_road->add_point();
                point_new->CopyFrom(msg->point(inliers->indices.at(i)));
            }
//            for (unsigned int i = 0; i < msg->height() * msg->width(); i++){
//                if ((float) msg->point(i).x() > roi_x_min && (float) msg->point(i).x() < roi_x_max
//                    && (float) msg->point(i).y() > roi_y_min && (float) msg->point(i).y() < roi_y_max
//                    && (float) msg->point(i).z() > roi_z_min && (float) msg->point(i).z() < roi_z_max){
//                    auto* point_new = msg_road->add_point();
//                    point_new->CopyFrom(msg->point(i));
//                }
//            }
            msg_road->mutable_header()->set_sequence_num(seq++);
            writer->Write(msg_road);
            auto finish = cyber::Time::Now();
            cout << finish - start << endl;
            return apollo::cyber::OK();
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr RoadDetectComponent::ConvertToPCL(std::shared_ptr<PointCloud> msg,
                float roi_x_min, float roi_x_max, float roi_y_min, float roi_y_max,
                float roi_z_min, float roi_z_max){
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

            cloud->width = msg->width();
            cloud->height = msg->height();
            cloud->points.resize (cloud->width * cloud->height);
            for (unsigned int i = 0; i < msg->height() * msg->width(); i++){
                if ((float) msg->point(i).x() > roi_x_min && (float) msg->point(i).x() < roi_x_max
                    && (float) msg->point(i).y() > roi_y_min && (float) msg->point(i).y() < roi_y_max
                    && (float) msg->point(i).z() > roi_z_min && (float) msg->point(i).z() < roi_z_max){
                    cloud->points[i].x = (float) msg->point(i).x();
                    cloud->points[i].y = (float) msg->point(i).y();
                    cloud->points[i].z = (float) msg->point(i).z();
                }
            }
            return cloud;
        };
    }
}