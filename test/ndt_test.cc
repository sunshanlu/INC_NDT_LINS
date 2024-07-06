#include "BagIO.h"
#include "IncNdt.h"

using namespace inc_ndt_lins;

int main(int argc, char **argv) {
    BagIO io("/media/rookie-lu/新加卷/Dataset/ULHK/test2");
    io.SetPointCloudCallback([](PointCloud::Ptr cloud) { return false; }, "/velodyne_points_0");

    auto target_cloud = VoxelCloud(io.ReadPointCloud());
    auto source_cloud = VoxelCloud(io.ReadPointCloud());

    IncNdt::Options options = {1.0, 1.0, 100000, 5.0, IncNdt::NearbyType::NEARBY6};
    IncNdt inc_ndt(options);

    SE3d init_pose;
    inc_ndt.AddCloud(target_cloud);
    int inlier_num = inc_ndt.AlignG2O(source_cloud, init_pose);

    return 0;
}