#include <thread>

#include "Viewer.h"

using namespace std::chrono_literals;

NAMESPACE_BEGIN

/// 可视化器构造
Viewer::Viewer()
    : leaf_size_(0.5)
    , resq_stop(false)
    , has_cloud_(false) {
    local_map_ = pcl::make_shared<PointCloud>();
    pcl_viewer_ = pcl::make_shared<PCLViewer>();
    pcl_viewer_->addCoordinateSystem(10, "world");
}

/**
 * @brief 可视化器运行主逻辑
 *
 */
void Viewer::Run() {
    while (!resq_stop.load()) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!has_cloud_) {
            std::this_thread::sleep_for(10ms);
            continue;
        }
        pcl_viewer_->spinOnce(30);
    }
}

/**
 * @brief 设置位姿和点云
 *
 * @param pose  输入的位姿信息
 * @param cloud 输入的点云信息
 */
void Viewer::SetPoseAndCloud(const SE3d &pose, PointCloud::Ptr cloud) {

    PointCloud::Ptr cloud_out = VoxelCloud(cloud, leaf_size_);
    *local_map_ += *cloud_out;
    local_map_ = VoxelCloud(local_map_, leaf_size_);
    Eigen::Affine3f Twv;
    Twv.matrix() = pose.matrix().cast<float>();
    pcl::visualization::PointCloudColorHandlerGenericField<PointT> field_color(local_map_, "z");

    {
        std::lock_guard<std::mutex> lock(mutex_);
        pcl_viewer_->removePointCloud("local_map");
        pcl_viewer_->removeCoordinateSystem("vehicle");
        pcl_viewer_->addPointCloud<PointT>(local_map_, field_color, "local_map");
        pcl_viewer_->addCoordinateSystem(5, Twv, "vehicle");
        has_cloud_ = true;
        pcl_viewer_->spinOnce(1);
    }

    if (local_map_->size() > 600000)
        leaf_size_ *= 1.26;
}

/// @brief 请求停止
void Viewer::RequestStop() { resq_stop.store(true); }

NAMESPACE_END