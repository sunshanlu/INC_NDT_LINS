#pragma once

#include "Common.hpp"
#include "ESKF.h"
#include "Tracker.h"

NAMESPACE_BEGIN

class LooselyLio {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<LooselyLio> Ptr;

    LooselyLio(SE3d Tli)
        : ndt_lo_(nullptr)
        , eskf_(nullptr)
        , Tli_(std::move(Tli))
        , first_se3_(true) {}

    /// 添加IMU数据信息，里面包含imu的时间戳信息
    void AddIMU(const IMU::Ptr &imu);

    // 添加点云数据信息
    void AddCloud(const FullPointCloud::Ptr &cloud, const double &laser_start_time);

    /// 设置ndt里程计
    void SetNdtLo(Tracker::Ptr ndt_lo) { ndt_lo_ = std::move(ndt_lo); }

    /// 设置ESKF
    void SetESKF(ESKF::Ptr eskf) { eskf_ = std::move(eskf); }

private:
    /// 使用位姿差值的方式进行点云去畸变
    PointCloud::Ptr UndistortCloud(const FullPointCloud::Ptr &cloud, const double &laser_start_time);

    /// 对位姿进行差值
    bool DifferencePose(const double &point_stamp, SE3d &Twl_t);

    /// 向外部发送导航状态
    void PublishNavState();

    /// 配准
    void Align();

    std::deque<FullPointCloud::Ptr> laser_buffer_; ///<输入的全量点云信息缓冲区
    std::deque<double> start_time_buffer_;         ///< 雷达开始时间戳缓冲区
    std::deque<double> end_time_buffer_;           ///< 雷达结束时间戳缓冲区

    Tracker::Ptr ndt_lo_; ///< 基于inc_ndt的激光里程计
    ESKF::Ptr eskf_;      ///< 误差状态卡尔曼滤波器
    SE3d Tli_;            ///< 雷达系到IMU系的变换
    bool first_se3_;      ///< 是否是第一次添se3观测信息

    std::vector<double> stamp_query_; ///< 时间戳查询区
    std::vector<SE3d> Twi_query_;     ///< 时间戳对应位姿查询区
    PointCloud::Ptr undistort_cloud_; ///< 去畸变后的点云信息
};

NAMESPACE_END