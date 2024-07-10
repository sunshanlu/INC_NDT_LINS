#pragma once
#include <atomic>
#include <mutex>

#include <pcl/visualization/pcl_visualizer.h>

#include "Common.hpp"

NAMESPACE_BEGIN

class Viewer {
public:
    typedef std::shared_ptr<Viewer> Ptr;
    typedef pcl::visualization::PCLVisualizer PCLViewer;

    Viewer();

    /// 可视化线程运行主逻辑
    void Run();

    /// 设置位姿和点云
    void SetPoseAndCloud(const SE3d &pose, PointCloud::Ptr cloud);

    /// 外部请求停止
    void RequestStop();

private:
    std::mutex mutex_;           ///< 可视化器互斥锁
    PCLViewer::Ptr pcl_viewer_;  ///< pcl的点云可视化器
    PointCloud::Ptr local_map_;  ///< 可视化的局部地图
    double leaf_size_;           ///< 体素滤波器的栅格大小
    std::atomic<bool> resq_stop; ///< 是否请求停止
    bool has_cloud_;             ///< 是否存在点云
};

NAMESPACE_END