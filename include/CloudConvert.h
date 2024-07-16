#pragma once
#include "Common.hpp"

NAMESPACE_BEGIN

/// 点云转换类，将不同的点云信息转换为全量点云
class CloudConvert {
public:
    typedef std::shared_ptr<CloudConvert> Ptr;

    /// 点云转换类可以转换的雷达类型
    enum class CloudType {
        VELO, ///< velodyne 雷达
        OUST, ///< OUST 雷达
        AVIA  ///< 大疆Avia固态雷达
    };

    /// 点云转换类配置项
    struct Options {
        CloudType cloud_type_; ///< 点云类型
        int point_filter_;     ///< 跳点数目，用来控制点云大小
        float time_scale_;     ///< 时间戳与s的比例
        int num_scans_;        ///< 多线雷达的线数
    };

    CloudConvert(Options options)
        : options_(std::move(options)) {}

    // clang-format off
    template <typename T>
    void Process(const T &cloud_msg, FullPointCloud::Ptr &pcl_out);
    // clang-format on

    /// 将全量点云转换为普通点云，用于imu失败的ndt
    static PointCloud::Ptr Full2PointCloud(const FullPointCloud::Ptr &cloud);

private:
    /// velodyne雷达点云转换处理逻辑
    void VelodyneHandler(const PointCloudPtr &cloud_msg, FullPointCloud::Ptr &pcl_out);

    /// OUST雷达点云转换处理逻辑
    void OustHandler(const PointCloudPtr &cloud_msg, FullPointCloud::Ptr &pcl_out);

    /// Livox雷达点云转换处理逻辑
    void LivoxHandler(const LivoxCloud::SharedPtr &cloud_msg, FullPointCloud::Ptr &pcl_out);

    Options options_; ///< 点云转换类配置项
};

NAMESPACE_END