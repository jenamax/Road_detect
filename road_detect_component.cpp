#include "road_detect_component.h"
#include <iostream>
#include <vector>


using std::cout;
using std::endl;
using pcl::PointXYZ;
using apollo::common::PointENU;
using std::vector;

double x = 0;
double y = 0;

namespace apollo {
    namespace road_detect {

        double dist(vector<double> p1, vector<double> p2){
            return (p1.at(0) - p2.at(0)) * (p1.at(0) - p2.at(0)) + (p1.at(1) - p2.at(1)) * (p1.at(1) - p2.at(1));
        }

        vector<double> lineFromPoints(vector<double> p1, vector<double> p2){
            double k = (p1.at(1) - p2.at(1)) / (p1.at(0) - p2.at(0));
            double b = p1.at(1) - k * p1.at(0);

            vector<double> cof;
            cof.push_back(k);
            cof.push_back(b);
            return cof;
        }

        void GnssCallback(const std::shared_ptr<apollo::localization::LocalizationEstimate> &gps_msg) {
            LocalizationEstimate gps = *gps_msg;
            x = gps.pose().position().x();
            y = gps.pose().position().y();
        }

        bool RoadDetectComponent::Init() {
            writer = node_->CreateWriter<PointCloud>("/apollo/road_pcl");
            gnss_listener = node_->CreateReader<LocalizationEstimate>(
                            "/apollo/localization/pose", GnssCallback);
            //map = apollo::hdmap::CreateMap("map/sim_map.bin");
            map = apollo::hdmap::HDMapUtil::BaseMapPtr();
            return apollo::cyber::OK();
        }

        bool RoadDetectComponent::Proc(const std::shared_ptr<PointCloud> &msg) {
            auto start = cyber::Time::Now();
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            std::vector<RoadInfoConstPtr> roads;
            const std::shared_ptr<PointENU> cur_pos = std::make_shared<PointENU>();
            cur_pos->set_x(x);
            cur_pos->set_y(y);
            map->GetRoads(*cur_pos, 10, &roads);
            cout << x << " " << y << endl;

            std::vector<std::vector<double>> left_edge_points;
            std::vector<std::vector<double>> right_edge_points;
            double kl = 0, kr = 0, br = 0, bl = 0;
            if (roads.size() > 0) {
                for (const auto &road : roads) {
                    for (const auto &section : road->road().section()) {
                        for (const auto &edge : section.boundary().outer_polygon().edge()) {
                            for (const auto &segment : edge.curve().segment()) {
                                for (const auto &point : segment.line_segment().point()) {
                                    std::vector<double> tmp;
                                    tmp.push_back(point.x());
                                    tmp.push_back(point.y());
                                    if (edge.type() == 2) {
                                        left_edge_points.push_back(tmp);
                                    } else if (edge.type() == 3) {
                                        right_edge_points.push_back(tmp);
                                    }
                                }
                            }
                        }
                    }
                }

                vector<double> cur_pose;
                cur_pose.push_back(x);
                cur_pose.push_back(y);

                std::sort(left_edge_points.begin(), left_edge_points.end(), [cur_pose](const vector<double> lhs,
                                                                                       const vector<double> rhs) {
                    return dist(cur_pose, lhs) < dist(cur_pose, rhs);
                });
                std::sort(right_edge_points.begin(), right_edge_points.end(), [cur_pose](const vector<double> lhs,
                                                                                         const vector<double> rhs) {
                    return dist(cur_pose, lhs) < dist(cur_pose, rhs);
                });

                vector<double> left_edge = lineFromPoints(left_edge_points.at(0), left_edge_points.at(1));
                vector<double> right_edge = lineFromPoints(right_edge_points.at(0), right_edge_points.at(1));

                kl = left_edge.at(0);
                kr = right_edge.at(0);
                bl = left_edge.at(1);
                br = right_edge.at(1);
            }


            cloud = ConvertToPCL(msg);
            cout << x << " " << y << endl;
            pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
            pcl::SACSegmentation<pcl::PointXYZ> seg;
            seg.setOptimizeCoefficients (true);
            seg.setModelType (pcl::SACMODEL_PLANE);
            seg.setMethodType (pcl::SAC_RANSAC);
            seg.setDistanceThreshold (0.2);
            seg.setInputCloud (cloud);
            seg.segment (*inliers, *coefficients);

            cout << inliers->indices.size()<< endl;

            pcl::PointCloud<pcl::PointXYZ>::Ptr plane_points(new pcl::PointCloud <pcl::PointXYZ>);
            plane_points->points.resize(inliers->indices.size());

//            pcl::PointXYZ cluster_start_point;
//            for (unsigned int i = 0; i < inliers->indices.size(); i++){
//                plane_points->points[i].x = cloud->points[inliers->indices.at(i)].x;
//                plane_points->points[i].y = cloud->points[inliers->indices.at(i)].y;
//                plane_points->points[i].z = cloud->points[inliers->indices.at(i)].z;
//                if (cloud->points[inliers->indices.at(i)].y > -0.2 && cloud->points[inliers->indices.at(i)].y < 0.2){
//                    cluster_start_point = cloud->points[inliers->indices.at(i)];
//                }
//            }

            std::shared_ptr<PointCloud> msg_road;
            msg_road = std::make_shared<PointCloud>();
            msg_road->mutable_point()->Reserve(240000);
            msg_road->Clear();
            msg_road->mutable_header()->set_timestamp_sec(msg->measurement_time());
            msg_road->mutable_header()->set_frame_id(msg->header().frame_id());
            msg_road->mutable_header()->set_lidar_timestamp(msg->measurement_time());
            msg_road->set_measurement_time(msg->measurement_time());
            msg_road->set_height(1);
            msg_road->set_width(inliers->indices.size());
            msg_road->set_is_dense(msg->is_dense());

            for (unsigned int i = 0; i < inliers->indices.size(); i++){
                auto* point_new = msg_road->add_point();
                x = msg->point(inliers->indices.at(i)).x();
                y = msg->point(inliers->indices.at(i)).y();
                if (roads.size() > 0 && (kl * x + bl < y && kr * x + br > y || kl * x + bl > y && kr * x + br < y)) {
                    point_new->CopyFrom(msg->point(inliers->indices.at(i)));
                }
            }

            msg_road->mutable_header()->set_sequence_num(seq++);
            writer->Write(msg_road);
            auto finish = cyber::Time::Now();
            cout << finish - start << endl;
            return apollo::cyber::OK();
        }
        pcl::PointCloud<pcl::PointXYZ>::Ptr RoadDetectComponent::ConvertToPCL(std::shared_ptr<PointCloud> msg,
                                                                              float roi_x_min, float roi_x_max,
                                                                              float roi_y_min, float roi_y_max,
                                                                              float roi_z_min, float roi_z_max) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud <pcl::PointXYZ>);

            cloud->width = msg->width();
            cloud->height = msg->height();
            cloud->points.resize(cloud->width * cloud->height);
            for (unsigned int i = 0; i < msg->height() * msg->width(); i++) {
                if ((float) msg->point(i).x() > roi_x_min && (float) msg->point(i).x() < roi_x_max
                    && (float) msg->point(i).y() > roi_y_min && (float) msg->point(i).y() < roi_y_max
                    && (float) msg->point(i).z() > roi_z_min && (float) msg->point(i).z() < roi_z_max) {
                    cloud->points[i].x = (float) msg->point(i).x();
                    cloud->points[i].y = (float) msg->point(i).y();
                    cloud->points[i].z = (float) msg->point(i).z();
                }
            }
            return cloud;
        }


        pcl::PointIndices::Ptr RoadDetectComponent::cluster(pcl::PointXYZ searchPoint, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {

            std::vector<int> pointIdxRadiusSearch;
            std::vector<float> pointRadiusSquaredDistance;

            pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
            kdtree.setInputCloud(cloud);

            float radius = 0.2;

            std::vector<pcl::PointXYZ> open_set;
            open_set.push_back(searchPoint);

            pcl::PointIndices::Ptr close_set (new  pcl::PointIndices);
            std::vector<int>::iterator it;
            bool point_closed = false;
            bool point_opened = false;
            while (open_set.size() != 0){
                if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
                    for (unsigned int i = 1; i < pointIdxRadiusSearch.size(); ++i) {
                        for (unsigned int j = 0; j < close_set->indices.size(); j++){
                            if (cloud->points[close_set->indices.at(j)].x == cloud->points[pointIdxRadiusSearch[i]].x &&
                                    cloud->points[close_set->indices.at(j)].y == cloud->points[pointIdxRadiusSearch[i]].y &&
                                    cloud->points[close_set->indices.at(j)].z == cloud->points[pointIdxRadiusSearch[i]].z){
                                point_closed = true;
                                break;
                            }
                        }
                        for (unsigned int j = 0; j < open_set.size(); j++){
                            if (open_set.at(j).x == cloud->points[pointIdxRadiusSearch[i]].x &&
                                    open_set.at(j).y == cloud->points[pointIdxRadiusSearch[i]].y &&
                                    open_set.at(j).z == cloud->points[pointIdxRadiusSearch[i]].z){
                                point_opened = true;
                                break;
                            }
                        }
                        if (!point_closed && !point_opened) {

                            open_set.push_back(cloud->points[pointIdxRadiusSearch[i]]);
                        }
                        point_closed = false;
                        point_opened = false;
                    }
                }
                unsigned int ind;
                for (unsigned int i = 0; i < cloud->points.size(); i++){
                    if (cloud->points[i].x == searchPoint.x && cloud->points[i].y == searchPoint.y && cloud->points[i].z == searchPoint.z){
                        ind = i;
                        break;
                    }
                }
                close_set->indices.push_back(ind);
                open_set.erase(open_set.begin());
                searchPoint = open_set.front();
            }
            return close_set;
        };

    }
}