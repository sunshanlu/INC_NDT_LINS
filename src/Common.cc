#include <pcl/filters/voxel_grid.h>

#include "Common.hpp"

NAMESPACE_BEGIN

PointCloud::Ptr VoxelCloud(PointCloud::Ptr cloud, float voxel_size) {
    pcl::VoxelGrid<PointT> vg;
    vg.setLeafSize(voxel_size, voxel_size, voxel_size);
    vg.setInputCloud(cloud);

    PointCloud::Ptr output = pcl::make_shared<PointCloud>();
    vg.filter(*output);
    return output;
}

NAMESPACE_END
