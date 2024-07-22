#include "BagIO.h"
#include "ESKF.h"
#include "IEKF.h"
#include "IEKFLio.h"
#include "IncNdt.h"
#include "Viewer.h"

using namespace inc_ndt_lins;

IncNdt::Options ndt_options = {1.0, 1.0, 100000, 5.0, IncNdt::NearbyType::NEARBY6};    ///< 增量ndt配置项
CloudConvert::Options convert_options = {CloudConvert::CloudType::VELO, 10, 1e-3, 32}; ///< 点云转换配置项
IMUStaticInit::Options imu_static_options = {false, 1000, 10, 9.81, true};             ///< IMU静态初始化配置项
IEKF::Options iekf_options = {5, 1e-3, 1e-4, 1e-8, true};

int main(int argc, char **argv) {
    Mat3d Ril;
    Vec3d til;
    Ril << 0, -1, 0, 1, 0, 0, 0, 0, 1;
    til << 0, 0, -0.28;

    IncNdt::Ptr inc_ndt = std::make_shared<IncNdt>(ndt_options);
    IEKF::Ptr iekf = std::make_shared<IEKF>(iekf_options);
    IMUStaticInit::Ptr imu_static_init = std::make_shared<IMUStaticInit>(imu_static_options);
    Viewer::Ptr viewer = std::make_shared<Viewer>();

    IEKFLio::Options lio_options = {0.5, 30 * M_PI / 180.0, SE3d(Ril, til).inverse()};
    IEKFLio::Ptr iekf_lio = std::make_shared<IEKFLio>(lio_options);
    iekf_lio->SetINCNDT(inc_ndt);
    iekf_lio->SetIEKF(iekf);
    iekf_lio->SetIMUInit(imu_static_init);
    iekf_lio->SetViewer(viewer);

    BagIO io("/media/rookie-lu/新加卷/Dataset/ULHK/test2");
    io.SetPointCloudCallback([&](FullPointCloud::Ptr cloud, double stamp) { iekf_lio->AddCloud(cloud, stamp); },
                             "/velodyne_points_0", convert_options)
        .SetIMUCallback([&](IMU::Ptr imu) { iekf_lio->AddIMU(imu); }, "/imu/data")
        .Go();
    return 0;
}
