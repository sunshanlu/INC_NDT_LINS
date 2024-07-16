#include <iostream>

#include "BagIO.h"

using namespace inc_ndt_lins;

CloudConvert::Options convert_options = {CloudConvert::CloudType::VELO, 1, 1e-3, 32};

int main(int argc, char **argv) {

    std::cout << std::fixed;
    std::cout <<std::setprecision(9);

    BagIO io("/media/rookie-lu/新加卷/Dataset/ULHK/test3");

    io.SetPointCloudCallback(
          [&](FullPointCloud::Ptr cloud, double stamp) { std::cout << "Las\t" << stamp << std::endl; },
          "/velodyne_points_0", convert_options)
        .SetIMUCallback([&](IMU::Ptr imu) { std::cout << "IMU\t" << imu->stamp_ << std::endl; }, "/imu/data")
        .Go();

    return 0;
}