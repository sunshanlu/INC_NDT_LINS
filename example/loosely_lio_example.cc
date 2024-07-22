#include "BagIO.h"
#include "ESKF.h"
#include "LooselyLio.h"
#include "Tracker.h"
#include "Viewer.h"

using namespace inc_ndt_lins;

IncNdt::Options ndt_options = {1.0, 1.0, 100000, 5.0, IncNdt::NearbyType::NEARBY6};
Tracker::Options track_options = {0.5, 30 * M_PI / 180.0, ndt_options};
CloudConvert::Options convert_options = {CloudConvert::CloudType::VELO, 10, 1e-3, 32};
IMUStaticInit::Options imu_static_options = {false, 1000, 10, 9.81, true};
int main(int argc, char **argv) {
    Mat3d Ril;
    Vec3d til;
    Ril << 0, -1, 0, 1, 0, 0, 0, 0, 1;
    til << 0, 0, -0.28;

    Viewer::Ptr viewer = std::make_shared<Viewer>();
    Tracker::Ptr ndt_lo = std::make_shared<Tracker>(track_options);
    ESKF::Ptr eskf = std::make_shared<ESKF>(imu_static_options);
    ndt_lo->SetViewer(viewer);

    LooselyLio::Ptr loosely_lio = std::make_shared<LooselyLio>(SE3d(Ril, til).inverse());
    loosely_lio->SetESKF(eskf);
    loosely_lio->SetNdtLo(ndt_lo);

    BagIO io("/media/rookie-lu/新加卷/Dataset/ULHK/test3");
    io.SetPointCloudCallback(
          [&](FullPointCloud::Ptr cloud, double stamp) {
              SE3d Twi;
              loosely_lio->AddCloud(cloud, stamp);
          },
          "/velodyne_points_0", convert_options)
        .SetIMUCallback([&](IMU::Ptr imu) { loosely_lio->AddIMU(imu); }, "/imu/data")
        .Go();

    return 0;
}