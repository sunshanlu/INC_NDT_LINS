#include <thread>

#include "BagIO.h"
#include "CloudConvert.h"
#include "IncNdtLo.h"
#include "Viewer.h"

using namespace inc_ndt_lins;
 
IncNdt::Options ndt_options = {1.0, 1.0, 100000, 5.0, IncNdt::NearbyType::NEARBY6};
IncNdtLo::Options track_options = {0.5, 5 * M_PI / 180.0};
CloudConvert::Options convert_options = {CloudConvert::CloudType::VELO, 3, 1e-3, 32};

int main(int argc, char **argv) {
    auto viewer = std::make_shared<Viewer>();
    auto ndt_lo = std::make_shared<IncNdtLo>(track_options);
    auto inc_ndt = std::make_shared<IncNdt>(ndt_options);
    ndt_lo->SetViewer(viewer);
    ndt_lo->SetIncNdt(inc_ndt);

    BagIO io("/media/rookie-lu/新加卷/Dataset/ULHK/test3");
    io.SetPointCloudCallback(
          [&](FullPointCloud::Ptr cloud, double stamp) {
              SE3d Twl;
              PointCloud::Ptr pcl_cloud = CloudConvert::Full2PointCloud(cloud);
              ndt_lo->AddCloud(pcl_cloud, Twl);
          },
          "/velodyne_points_0", convert_options)
        .Go();

    return 0;
}