#include "BagIO.h"
#include "CloudConvert.h"
#include "IMUPreintegration.h"
#include "PreinteLio.h"
#include "Viewer.h"

using namespace inc_ndt_lins;

IncNdt::Options ndt_options = {1.0, 1.0, 100000, 5.0, IncNdt::NearbyType::NEARBY6};    ///< 增量ndt配置项
CloudConvert::Options convert_options = {CloudConvert::CloudType::VELO, 10, 1e-3, 32}; ///< 点云转换配置项
IMUStaticInit::Options imu_static_options = {false, 1000, 10, 9.81, false};             ///< IMU静态初始化配置项

int main(int argc, char **argv) {
    Mat3d Ril;
    Vec3d til;
    Ril << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    til << 0, 0, 0.28;

    PreinteLio::Options lio_options = {false, SE3d(Ril, til).inverse(), 0.3, 3 * M_PI / 180.0, 0.1, 2 * M_PI / 180.0};

    IncNdt::Ptr inc_ndt = std::make_shared<IncNdt>(ndt_options);
    IMUStaticInit::Ptr imu_static_init = std::make_shared<IMUStaticInit>(imu_static_options);
    Viewer::Ptr viewer = std::make_shared<Viewer>();
    PreinteLio::Ptr preint_lio = std::make_shared<PreinteLio>(lio_options);
    preint_lio->SetIncNdt(inc_ndt);
    preint_lio->SetImuInit(imu_static_init);
    preint_lio->SetViewer(viewer);

    BagIO io("/media/rookie-lu/新加卷/Dataset/NCLT/20130110");
    io.SetPointCloudCallback([&](FullPointCloud::Ptr cloud, double stamp) { preint_lio->AddCloud(cloud, stamp); },
                             "points_raw", convert_options)
        .SetIMUCallback([&](IMU::Ptr imu) { preint_lio->AddIMU(imu); }, "imu_raw")
        .Go();
    return 0;
}
