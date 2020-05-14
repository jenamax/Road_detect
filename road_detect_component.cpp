#include "road_detect_component.h"
#include <iostream>
#include <vector>
#include <math.h>
#include <numeric>
#include <fstream>
#include <iterator>
#include <algorithm>
#include <valarray>
#include <complex>
#include "DigitalFilters-master/DigitalFilters.h"

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
unsigned int nScanRings = 16;
double factor = (nScanRings - 1) / (upper_bound_angle - lower_bound_angle);


namespace apollo {
    namespace road_detect {

        double median(vector<double> a){
            vector<double> copy_a;
            std::copy(a.begin(), a.end(), std::back_inserter(copy_a));
            int n = a.size();
            std::sort(copy_a.begin(), copy_a.end());

            // check for even case
            if (n % 2 != 0)
                return (double)copy_a[n/2];

            return (double)(copy_a[(n-1)/2] + copy_a[n/2])/2.0;
        }

        double mean(vector<double> a){
            double sum = 0;
            for (unsigned int i = 0; i < a.size(); i++){
                sum += a[i];
            }
            return sum / a.size();
        }

        double stdev(vector<double> a){
            double mean_val = mean(a);
            vector<double> dif;
            for (unsigned int i = 0; i < a.size(); i++){
                dif.push_back(fabs(a[i] - mean_val));
            }
            return mean(dif);
        }

        vector<double> line_fit_lsm(vector<double> x, vector<double> y){
            double xsum = 0,x2sum = 0,ysum = 0,xysum = 0, a, b;
            int n = x.size();
            for (int i = 0;i < n; i++)
            {

                xsum = xsum + x[i];
                ysum = ysum + y[i];
                x2sum = x2sum + pow(x[i],2);
                xysum = xysum + x[i] * y[i];
            }

            a = (n * xysum - xsum * ysum) / (n * x2sum - xsum * xsum);
            b =(x2sum*ysum-xsum*xysum)/(x2sum*n-xsum*xsum);
            return vector<double> {a, b};
        }

        vector<double> line_ransac(vector<double> x, vector<double> y, double thrsh, double iter){
            int p1, p2, inliers_num, max_inliers = 0;
            double x1, x2, y1, y2, k, b, d;
            vector<double> line_coef;
            for (int i = 0; i < iter; i++){
                p1 = std::rand() % x.size();
                p2 = std::rand() % x.size();
                if (p1 == p2){
                    iter += 1;
                    continue;
                }
                x1 = x[p1];
                y1 = y[p1];
                x2 = x[p2];
                y2 = y[p2];

                k = atan((y2 - y1) / (x2 - x1));
                b = y1 - k * x1;

                inliers_num = 0;
                for (unsigned int j = 0; j < x.size(); j++){
                    d = abs(y[j] - k * x[j] - b) / sqrt(k * k + 1);
                    if (d < thrsh){
                        inliers_num++;
                    }
                }
                if (inliers_num > max_inliers){
                    line_coef = {k, b};
                }
            }
            return line_coef;
        }

        double point_angle(pcl::PointXYZ point){
            double x = point.x;
            double y = point.y;
            double z = point.z;

            return atan(z / sqrt(x*x + y*y));
        }

        int scan_line_num(pcl::PointXYZ point){
            double  angle = point_angle(point);
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
            pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
            pcl::SACSegmentation<pcl::PointXYZ> seg;
            seg.setOptimizeCoefficients (true);
            seg.setModelType (pcl::SACMODEL_PLANE);
            seg.setMethodType (pcl::SAC_RANSAC);
            seg.setDistanceThreshold (0.3);
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
            vector<vector<double>> lidar_angles;
            vector<vector<double>> dist;
            for (unsigned int i = 0; i < nScanRings; i++){
                scan_lines.push_back({});
                lidar_angles.push_back({});
                dist.push_back({});
            }

            //double roi_x_min = 0, roi_x_max = 30, roi_y_min = -20, roi_y_max = 20;
            double x, y, z, angle;//, x_i1, y_i1, z_i1, x_i2, y_i2, z_i2, s, x1, y1, z1;


            int line_num = 0;
            for (unsigned int i = 0; i < inliers->indices.size(); i++){
                x = msg->point(inliers->indices[i]).x();
                y = msg->point(inliers->indices[i]).y();
                z = msg->point(inliers->indices[i]).z();
                angle = atan2(y, x) ; //+ M_PI / 2;
               // if (angle > 0 && angle < M_PI) {
                line_num = scan_line_num(cloud->points[inliers->indices[i]]);
                scan_lines[line_num].push_back(inliers->indices[i]);
                lidar_angles[line_num].push_back(angle);
                dist[line_num].push_back(sqrt(x*x + y*y + z*z));
                //}
            }
            for (unsigned int i = 0; i < nScanRings; i++) {
                if (scan_lines[i].size() < 200) {
                    continue;
                }
                std::pair <double, int> angle_line[scan_lines[i].size()];
                std::pair <double, double> angle_dist[scan_lines[i].size()];
                for(unsigned int j = 0 ; j < scan_lines[i].size(); j++){
                    angle_line[j] = std::make_pair(lidar_angles[i][j], scan_lines[i][j]);
                    angle_dist[j] = std::make_pair(lidar_angles[i][j], dist[i][j]);
                }
                std::sort(angle_line, angle_line+scan_lines[i].size());
                std::sort(angle_dist, angle_dist+scan_lines[i].size());
                for (unsigned int j = 0; j < scan_lines[i].size(); j++){
                    scan_lines[i][j] = angle_line[j].second;
                    dist[i][j] = angle_dist[j].second;
                    lidar_angles[i][j] = angle_line[j].first;
                }
            }

            vector<vector<double>> d_dist;
            vector<vector<unsigned int>> peaks;
            vector<vector<unsigned int>> peaks_point;
            vector<vector<unsigned int>> peaks_dist;
            std::ofstream ddist_file, dist_file, bord_file, peaks_file, ang_file;
            ddist_file.open ("/apollo/modules/road_detect/d_dist0.txt", std::ofstream::trunc);
            ang_file.open ("/apollo/modules/road_detect/ang.txt", std::ofstream::trunc);
            dist_file.open ("/apollo/modules/road_detect/dist0.txt", std::ofstream::trunc);
            peaks_file.open ("/apollo/modules/road_detect/peaks.txt", std::ofstream::trunc);

            constexpr float cutoffFrequeny = 20;

            LowPassFilter lpf(0.01, 2 * M_PI * cutoffFrequeny);
            for (unsigned int i = 0; i < nScanRings; i++) {
                d_dist.push_back({});
                peaks.push_back({});
                peaks_point.push_back({});
                peaks_dist.push_back({});
                if (dist[i].size() == 0) {
                    continue;
                }
                for (unsigned int j = 0; j < dist[i].size() - 1; j++){
                    d_dist[i].push_back(lpf.update(fabs(dist[i][j] - dist[i][j + 1])));
                    ddist_file << d_dist[i][j] << " ";
                    dist_file << dist[i][j] << " ";
                    ang_file << lidar_angles[i][j] << " ";
                }
                ddist_file << "\n";
                dist_file << "\n";
                ang_file << "\n";
            }

            ddist_file.close();
            dist_file.close();

            vector<double> threshold = {0.025, 0.03, 0.03, 0.05, 0.07, 0.1, 0.12, 0.2};

            double med;
            for (unsigned int i = 0; i < nScanRings; i++) {
                if (d_dist[i].size() == 0) {
                    continue;
                }
                //med = median(d_dist[i]);
                for (unsigned int j = 0; j < d_dist[i].size(); j++){
                    if (d_dist[i][j] > threshold[i]){
                        //cout << j<< endl;
                        peaks[i].push_back(j);
                        peaks_point[i].push_back(scan_lines[i][j]);
                        peaks_file << j << " ";
                    }
                }
                peaks_file << "\n";
            }
            peaks_file.close();
            for (unsigned int i = 0; i < nScanRings; i++) {
                if (peaks[i].size() == 0) {
                    continue;
                }
                for (unsigned int j = 0; j < peaks[i].size() - 1; j++){
                    peaks_dist[i].push_back(abs(lidar_angles[i][peaks[i][j + 1]] - lidar_angles[i][peaks[i][j]]));
                }
            }

            vector<unsigned int> bord1;
            vector<unsigned int> bord2;
            const unsigned int no_bord_marker = 10000;

            vector<double> dist_part;
            double min_med = 100000;
            for (unsigned int i = 0; i < nScanRings; i++) {
                if (peaks[i].size() == 0) {
                    bord1.push_back(no_bord_marker);
                    bord2.push_back(no_bord_marker);
                    continue;
                }
                min_med = 100000;
                for (unsigned int j = 0; j < peaks[i].size() - 1; j++){
                    if (peaks_dist[i][j] < *std::max_element(peaks_dist[i].begin(), peaks_dist[i].end()) * 4 / 5){
                        continue;
                    }
                    for (unsigned int k = peaks[i][j]; k < peaks[i][j + 1]; k++){
                        dist_part.push_back(d_dist[i][k]);
                    }
                    med = mean(dist_part);
                    if (med < min_med){
                        if (bord1.size() > i) {
                            bord1[i] = peaks[i][j];
                            bord2[i] = peaks[i][j + 1];
                        }
                        else{
                            bord1.push_back(peaks[i][j]);
                            bord2.push_back(peaks[i][j + 1]);
                        }
                        min_med = med;
                    }
                    dist_part.erase(dist_part.begin(), dist_part.end());
                }
            }

            bord_file.open ("/apollo/modules/road_detect/bord.txt", std::ofstream::trunc);
            for (unsigned int i = 0; i < nScanRings; i++) {
                bord_file << bord1[i] << " " << bord2[i] << "\n";
            }
            bord_file.close();

            for (unsigned int i = 0; i < nScanRings; i++){
                cout << scan_lines[i].size() << " " ;
            }
            cout << endl;

            vector <double> bord1_x, bord1_y;
            vector <double> bord2_x, bord2_y;
            vector<unsigned int> border_points_ind;
            for (unsigned int i = 2; i < nScanRings; i++) {
                if (dist[i].size() == 0){
                    continue;
                }
                if (bord1[i] != no_bord_marker){
                    border_points_ind.push_back(scan_lines[i][bord1[i]]);
                    bord1_x.push_back(msg->point(scan_lines[i][bord1[i]]).x());
                    bord1_y.push_back(msg->point(scan_lines[i][bord1[i]]).y());
                }
                if (bord2[i] != no_bord_marker){
                    border_points_ind.push_back(scan_lines[i][bord2[i]]);
                    bord2_x.push_back(msg->point(scan_lines[i][bord2[i]]).x());
                    bord2_y.push_back(msg->point(scan_lines[i][bord2[i]]).y());
                }
            }


            double yb;
            double std_thrsh = 10;

            double outlier_thrsh = 3;

            double mean_y1 = mean(bord1_y), std_y1 = stdev(bord1_y);

            for (unsigned int i = 0; i < bord1_y.size(); i++) {
                if (fabs(bord1_y[i] - mean_y1) > outlier_thrsh * std_y1) {
                    bord1_x.erase(bord1_x.begin() + i);
                    bord1_y.erase(bord1_y.begin() + i);
                }
            }

            bool bord1_exist = stdev(bord1_y) < std_thrsh;
            double ransac_thrs = 0.5;
            int ransac_iter = 50;

            bool ransac = true;
            if (bord1_exist) {
                vector<double> bord1_line_coef;
                if (ransac)
                    bord1_line_coef = line_ransac(bord1_x, bord1_y, ransac_thrs, ransac_iter);
                else
                    bord1_line_coef = line_fit_lsm(bord1_x, bord1_y);
                for (double xb = 2; xb < 30; xb += 0.5) {
                    yb = bord1_line_coef[0] * xb + bord1_line_coef[1];
                    auto *point_new = msg_road->add_point();
                    point_new->set_x(xb);
                    point_new->set_y(yb);
                    point_new->set_z(msg->point(inliers->indices[0]).z());
                }
            }

            double mean_y2 = mean(bord2_y), std_y2 = stdev(bord2_y);
            cout << "mean_y borders " << mean_y1 << " " << mean_y2 << endl;
            cout << "std_y borders " << std_y1 << " " << std_y2 << endl;
            for (unsigned int i = 0; i < bord2_y.size(); i++) {
                if (fabs(bord2_y[i] - mean_y2) > outlier_thrsh * std_y2) {
                    bord2_x.erase(bord2_x.begin() + i);
                    bord2_y.erase(bord2_y.begin() + i);
                }
            }
            bool bord2_exist = stdev(bord2_y) < std_thrsh;
            if (bord2_exist) {
                vector<double> bord2_line_coef;
                if (ransac)
                    bord2_line_coef = line_ransac(bord2_x, bord2_y, ransac_thrs, ransac_iter);
                else
                    bord2_line_coef = line_fit_lsm(bord2_x, bord2_y);

                for (double xb = 2; xb < 30; xb += 0.5) {
                    yb = bord2_line_coef[0] * xb + bord2_line_coef[1];
                    auto *point_new = msg_road->add_point();
                    point_new->set_x(xb);
                    point_new->set_y(yb);
                    point_new->set_z(msg->point(inliers->indices[0]).z());
                }
            }
//            vector<unsigned int> road_points_ind;
//            for (unsigned int i = 0; i < nScanRings; i++) {
//                if (bord1[i] == no_bord_marker || bord2[i] == no_bord_marker){
//                    continue;
//                }
//                for (unsigned int j = bord1[i]; j < bord2[i]; i++){
//                    road_points_ind.push_back(scan_lines[i][j]);
//                }
//            }


            for (unsigned int i = 0; i < border_points_ind.size(); i++) {
                auto *point_new = msg_road->add_point();
                point_new->CopyFrom(msg->point(border_points_ind[i]));
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