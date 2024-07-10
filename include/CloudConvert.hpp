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

private:
    /// velodyne雷达点云转换处理逻辑
    void VelodyneHandler(const PointCloudPtr &cloud_msg, FullPointCloud::Ptr &pcl_out);

    /// OUST雷达点云转换处理逻辑
    void OustHandler(const PointCloudPtr &cloud_msg, FullPointCloud::Ptr &pcl_out);

    /// Livox雷达点云转换处理逻辑
    void LivoxHandler(const LivoxCloud::SharedPtr &cloud_msg, FullPointCloud::Ptr &pcl_out);

    Options options_; ///< 点云转换类配置项
};

// clang-format off
/**
 * @brief 模版参数为PointCloudPtr的特例化参数模版，对应的是ros的PointCloud2类型
 * 
 * @tparam  
 * @param cloud_msg 输入的原始点云
 * @param pcl_out   输出的全量点云数据
 */
template <> 
void CloudConvert::Process<PointCloudPtr>(const PointCloudPtr &cloud_msg, FullPointCloud::Ptr &pcl_out) {
    switch (options_.cloud_type_) {
    case CloudType::VELO:
        VelodyneHandler(cloud_msg, pcl_out);
        break;
    case CloudType::OUST:
        OustHandler(cloud_msg, pcl_out);
        break;

    default:
        break;
    }
}
// clang-format on

/**
 * @brief 模版参数为大疆激光雷达CustomMsg的类型
 *
 * @tparam
 * @param cloud_msg 输入的原始点云
 * @param pcl_out   输出的全量点云数据
 */
template <>
void CloudConvert::Process<LivoxCloud::SharedPtr>(const LivoxCloud::SharedPtr &cloud_msg,
                                                  FullPointCloud::Ptr &pcl_out) {
    LivoxHandler(cloud_msg, pcl_out);
    return;
}

NAMESPACE_END