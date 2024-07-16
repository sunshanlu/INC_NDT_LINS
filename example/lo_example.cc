#include <thread>

#include "BagIO.h"
#include "CloudConvert.h"
#include "Tracker.h"
#include "Viewer.h"

using namespace inc_ndt_lins;

IncNdt::Options ndt_options = {1.0, 1.0, 100000, 5.0, IncNdt::NearbyType::NEARBY6};
Tracker::Options track_options = {0.5, 30 * M_PI / 180.0, ndt_options};
CloudConvert::Options convert_options = {CloudConvert::CloudType::VELO, 10, 1e-3, 32};

int main(int argc, char **argv) {
    auto viewer = std::make_shared<Viewer>();
    auto tracker = std::make_shared<Tracker>(track_options);
    tracker->SetViewer(viewer);
    // std::thread viewer_thread(&Viewer::Run, viewer);

    BagIO io("/media/rookie-lu/新加卷/Dataset/ULHK/test3");
    io.SetPointCloudCallback(
          [&](FullPointCloud::Ptr cloud, double stamp) {
              SE3d Twl;
              PointCloud::Ptr pcl_cloud = CloudConvert::Full2PointCloud(cloud);
              tracker->AddCloud(pcl_cloud, Twl);
          },
          "/velodyne_points_0", convert_options)
        .Go();

    // viewer->RequestStop();
    // viewer_thread.join();
    return 0;
}