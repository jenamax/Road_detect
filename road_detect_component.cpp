#include "road_detect_component.h"
#include <iostream>
#include <vector>
#include <math.h>


using std::cout;
using std::endl;
using pcl::PointXYZ;
using apollo::common::PointENU;
using std::vector;

double x = 0;
double y = 0;
double phi = 0;

double lower_bound_angle = -15;
double upper_bound_angle = 15;
int nScanRings = 16;
double factor = (nScanRings - 1) / (upper_bound_angle - lower_bound_angle);

namespace apollo {
    namespace road_detect {

        int scan_line_num(pcl::PointXYZ point){
            double x = point.x;
            double y = point.y;
            double z = point.z;

            double angle = atan(z / sqrt(x*x + y*y));
            return int(((angle * 180 / M_PI) - lower_bound_angle) * factor + 0.5);
        }

        bool RoadDetectComponent::Init() {
            writer = node_->CreateWriter<PointCloud>("/apollo/road_pcl");
            return apollo::cyber::OK();
        }

        bool RoadDetectComponent::Proc(const std::shared_ptr<PointCloud> &msg) {
            auto start = cyber::Time::Now();
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);


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

            cout << msg->height() * msg->width() << " " << inliers->indices.size()<< endl;

            pcl::PointCloud<pcl::PointXYZ>::Ptr plane_points(new pcl::PointCloud <pcl::PointXYZ>);
            plane_points->points.resize(inliers->indices.size());

            std::shared_ptr<PointCloud> msg_road;
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

            vector<vector<int>> scan_lines;

            for (int i = 0; i < 16; i++){
                scan_lines.push_back({});
            }

            for (unsigned int i = 0; i < inliers->indices.size(); i++){
                scan_lines.at(scan_line_num(cloud->points[inliers->indices[i]])).push_back(inliers->indices[i]);
            }

            for (int i = 0; i < 16; i++){
                cout << scan_lines[i].size() << " " ;
            }
            cout << endl;

            vector<double> threshold;
            vector<vector<double>> smoothness;
            for (int i = 0; i < 8; i++){
                smoothness.push_back({});
            }

            double x, y, z, x_i1, y_i1, z_i1, x_i2, y_i2, z_i2, s;

            vector<vector<int>> road_mask;
            for (int i = 0; i < 8; i++){
                road_mask.push_back({});
            }
            threshold = {152, 132, 112, 89, 69, 48, 28, 8};
            for (int i = 0; i < 8; i++) {
                for (unsigned int j = 2; j < scan_lines[i].size(); j++) {
                    x = msg->point(scan_lines[i][j]).x();
                    y = msg->point(scan_lines[i][j]).y();
                    z = msg->point(scan_lines[i][j]).z();

                    x_i1 = msg->point(scan_lines[i][j - 1]).x();
                    y_i1 = msg->point(scan_lines[i][j - 1]).y();
                    z_i1 = msg->point(scan_lines[i][j - 1]).z();

                    x_i2 = msg->point(scan_lines[i][j - 2]).x();
                    y_i2 = msg->point(scan_lines[i][j - 2]).y();
                    z_i2 = msg->point(scan_lines[i][j - 2]).z();

                    s = z - z_i1 / sqrt((x - x_i1)*(x - x_i1) + (y - y_i1)*(y - y_i1)) - z_i1 - z_i2 / sqrt((x_i2 - x_i1)*(x_i2 - x_i1) + (y_i2 - y_i1)*(y_i2 - y_i1));
                    smoothness[i].push_back(abs(s));
                    road_mask[i].push_back((int)(s > threshold[i]));
                }
            }

            vector<vector<int>> road_mask_averaged;
            for (int i = 0; i < 8; i++){
                road_mask.push_back({});
            }

            for (int i = 0; i < 8; i++) {
                for (unsigned int j = 2; j < scan_lines[i].size(); j++) {
                    x = msg->point(scan_lines[i][j]).x();
                    y = msg->point(scan_lines[i][j]).y();
                    z = msg->point(scan_lines[i][j]).z();

                    x_i1 = msg->point(scan_lines[i][j - 1]).x();
                    y_i1 = msg->point(scan_lines[i][j - 1]).y();
                    z_i1 = msg->point(scan_lines[i][j - 1]).z();

                    x_i2 = msg->point(scan_lines[i][j - 2]).x();
                    y_i2 = msg->point(scan_lines[i][j - 2]).y();
                    z_i2 = msg->point(scan_lines[i][j - 2]).z();

                    s = z - z_i1 / sqrt((x - x_i1)*(x - x_i1) + (y - y_i1)*(y - y_i1)) - z_i1 - z_i2 / sqrt((x_i2 - x_i1)*(x_i2 - x_i1) + (y_i2 - y_i1)*(y_i2 - y_i1));
                    if (abs(s) > threshold[i]) {
                        //cout << abs(s) << " " << threshold[i] << endl;
                        auto *point_new = msg_road->add_point();
                        point_new->CopyFrom(msg->point(scan_lines[i][j]));
                    }
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
    }
}